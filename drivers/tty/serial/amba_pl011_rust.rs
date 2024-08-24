// SPDX-License-Identifier: GPL-2.0

//! Driver for AMBA serial ports (PL011).
//!
//! Based on the C driver written by ARM Ltd/Deep Blue Solutions Ltd.

use core::{clone::Clone, convert::AsRef, ffi::{c_uchar, c_void}, ops::{Deref, DerefMut}, option::Option::Some, ptr::null, result::Result::Ok, u32};

use kernel::{
    amba, bindings, c_str, 
    define_amba_id_table, 
    device::{self, RawDevice}, 
    driver,
    driver_amba_id_table, 
    error::{code::*, Result}, 
    io_mem::IoMem, irq, 
    module_amba_driver, 
    module_amba_id_table,
    new_device_data, 
    prelude::{Box, *}, 
    serial:: {pl011_config::*, uart_console::{flags, Console, ConsoleOps}, uart_driver::UartDriver, uart_port::{uart_circ_empty, PortRegistration, UartPort, UartPortOps}},
    sync::UniqueArc, 
    types::ForeignOwnable
};

const UART_SIZE: usize = 0x200;
const UPIO_MEM: u32 = 2;
const UPIO_MEM32: u32 = 3;


const UART_DR_ERROR: u32 = UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE;
const UART_DUMMY_DR_RX:u32 = 1 << 16;

pub const UPF_BOOT_AUTOCONF: u64 = 1_u64 << 28;

pub(crate) const UART_NR: usize = 14;
const AMBA_MAJOR: i32 = 204;
const AMBA_MINOR: i32 = 64;
const AMBA_ISR_PASS_LIMIT: i32 = 256;
const DEV_NAME: &CStr = c_str!("ttyAMA");
const DRIVER_NAME: &CStr = c_str!("ttyAMA");

/// A static's struct with all uart_port
pub(crate) static mut PORTS: [Option<UartPort>; UART_NR] = [None; UART_NR];

/// This amba_uart_console static's struct
static AMBA_CONSOLE: Console = {
    let name:[i8; 16usize] =[
        't' as _, 't' as _, 'y' as _, 'A' as _, 'M' as _, 'A' as _, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    ];
    Console::new::<Pl011Console>(name,UART_DRIVER.as_ptr())
    .with_config(
        (flags::CON_PRINTBUFFER | flags::CON_ANYTIME) as _,
        -1,0,  0, 0, 0, 0,
    ) 
};

static mut IS_INIT: bool = false;

/// This uart_driver static's struct
pub(crate) static UART_DRIVER: UartDriver = UartDriver::new(
    &THIS_MODULE, 
    DRIVER_NAME, 
    DEV_NAME,
    &AMBA_CONSOLE,
    ).with_config(
        AMBA_MAJOR, 
        AMBA_MINOR, 
        UART_NR as _,
    );

/// This is Struct of pl011_console
struct Pl011Console;
impl Pl011Console {
    extern "C" fn pl011_console_putchar(port_ptr: *mut bindings::uart_port, ch: c_uchar) {
        let port = unsafe { *port_ptr };
        while PL011Device::pl011_read(port.membase, UART01X_FR as usize, port.iotype) & UART01X_FR_TXFF != 0 {
            unsafe { bindings::cpu_relax() }; // CPU 进入低功耗状态
        }
        PL011Device::pl011_write(ch as u32, port.membase, UART01X_DR as usize, port.iotype); // 将字符写入数据寄存器
    }

    fn pl011_console_get_options(uap: &UartPort, baud: &mut i32, parity: &mut i32, bits: &mut i32) {
        let port = unsafe { *uap.as_ptr() };
        if PL011Device::pl011_read(port.membase, UART011_CR as usize, port.iotype) & UART01X_CR_UARTEN != 0 {  // 检查 UART 是否启用
            let lcr_h = PL011Device::pl011_read(port.membase, UART011_LCRH as usize, port.iotype); // 读取行控制寄存器

            *parity = 'n' as i32;
            if lcr_h & UART01X_LCRH_PEN != 0 { // 检查是否启用奇偶校验
                if lcr_h & UART01X_LCRH_EPS != 0 {
                    *parity = 'e' as i32; // 启用偶校验
                } else {
                    *parity = 'o' as i32; // 启用奇校验
                }

                if (lcr_h & 0x60) == UART01X_LCRH_WLEN_7 {
                    *bits = 7; // 数据位为7
                } else {
                    *bits = 8; // 数据位为8
                }

                let ibrd = PL011Device::pl011_read(port.membase, UART011_IBRD as usize, port.iotype); // 读取整数波特率除数
                let fbrd = PL011Device::pl011_read(port.membase, UART011_FBRD as usize, port.iotype); // 读取小数波特率除数

                let pl011_data: Box<PL011Data> = unsafe {
                    <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
                };
                *baud = (port.uartclk * 4 / (64 * ibrd + fbrd)) as i32; // 计算波特率

                if pl011_data.vendor.oversampling { // 如果启用过采样
                    if PL011Device::pl011_read(port.membase, UART011_CR as usize, port.iotype) & ST_UART011_CR_OVSFACT != 0 {
                        *baud *= 2; // 波特率翻倍
                    }
                }
            }
        }
    }

    fn pl011_console_setup(co: &Console, options: *mut i8) -> Result<i32> {
        pr_info!("****pl011_console_setup****\n");
        let mut baud: i32 = 38400;  // 默认波特率
        let mut bits: i32 = 8; // 默认数据位
        let mut parity: i32 = 'n' as i32; // 默认无奇偶校验
        let mut flow: i32 = 'n' as i32; // 默认无流控

        let co_ptr = unsafe { &mut *co.as_ptr() };

        /*
         * 检查是否指定了无效的 UART 号，
         * 如果是，则搜索第一个可用的具有控制台支持的端口。
         */
        if co_ptr.index >= UART_NR as i16 {
            co_ptr.index = 0;
        }

        if unsafe { &PORTS[co_ptr.index as usize] }.is_none() {
            return Err(ENODEV);  // 如果端口不存在，返回 -ENODEV
        }
        let uap = &unsafe { &PORTS[co_ptr.index as usize] }.unwrap(); // 获取指定索引的端口
        let uap_ptr = unsafe { &mut *uap.as_ptr() };
        let pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(uap_ptr.private_data)
        };

        let dev = unsafe { device::Device::new(uap_ptr.dev) };

        /* 允许引脚复用并配置 */
        unsafe { bindings::pinctrl_pm_select_default_state(dev.raw_device()) }; 

        let clk = dev.clk_get().unwrap();
        clk.prepare_enable()?;  //准备时钟



        uap_ptr.uartclk = clk.get_rate() as u32; // 获取 UART 时钟频率

        if pl011_data.vendor.fixfixed_options {
            baud = pl011_data.fixed_baud as i32; // 如果固定波特率选项启用，使用固定波特率
        } else {
            if !options.is_null() {
                unsafe {
                    bindings::uart_parse_options(options,
                                                 &mut baud as *mut _,
                                                 &mut parity as *mut _,
                                                 &mut bits as *mut _,
                                                 &mut flow as *mut _); // 解析命令行选项
                }
            } else {
                Self::pl011_console_get_options(uap, &mut baud, &mut parity, &mut bits); // 获取控制台选项
            }
        }

        let ret = unsafe { bindings::uart_set_options(uap.as_ptr(), co.as_ptr(), baud, parity, bits, flow) }; // 设置 UART 选项

        if ret == 0 {
            Ok(0)
        } else {
            Err(ENODEV)
        }
    }
}

/// Implement supported `Pl011Console`'s operations here.
#[vtable]
impl ConsoleOps for Pl011Console {
    type Data = ();

    fn console_write(_co: &Console, _s: *const i8, _count: u32) {
        pr_info!("pl011 console_write ok");
        let co_ptr = unsafe { &*_co.as_ptr() };
        let uap = &unsafe { &PORTS[co_ptr.index as usize] }.unwrap();
        let uap_ptr = unsafe { &mut *uap.as_ptr() };
        let pl011_data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(uap_ptr.private_data)
        };
        let mut old_cr: u32 = 0;
        let mut new_cr: u32 = 0;
        let mut locked = true;
        let dev =  unsafe { device::Device::new(uap_ptr.dev) };
        let _= dev.clk_get().unwrap().prepare_enable();

        if uap_ptr.sysrq != 0 {
            locked = false;
        } else if unsafe { bindings::oops_in_progress != 0 } {
            // 如果有故障发生，尝试加锁
            unsafe { bindings::spin_lock(&uap_ptr.lock as *const _ as *mut _) };
        } else {
            unsafe { bindings::spin_lock(&uap_ptr.lock as *const _ as *mut _) };
        }

        if !pl011_data.vendor.always_enabled {
            old_cr = PL011Device::pl011_read(uap_ptr.membase, UART011_CR as usize, uap_ptr.iotype);
            new_cr = old_cr & !UART011_CR_CTSEN;
            PL011Device::pl011_write(new_cr, uap_ptr.membase, UART011_CR as usize, uap_ptr.iotype);
        }
        unsafe { bindings::uart_console_write(uap.as_ptr(), _s, _count, Some(Self::pl011_console_putchar)) }

        while (PL011Device::pl011_read(uap_ptr.membase, UART01X_FR as usize, uap_ptr.iotype) ^ pl011_data.vendor.inv_fr) 
               & pl011_data.vendor.fr_busy != 0 {
            unsafe { bindings::cpu_relax() };
        }
        if locked {
            unsafe { bindings::spin_unlock(&mut uap_ptr.lock) };
        }

        let _ = dev.clk_get().unwrap().disable();
    }

    fn console_read(_co: &Console, _s: *mut i8, _count: u32) -> Result<i32>{
        pr_info!("pl011 console_read ok");
        Ok(0)
    }

    fn console_match(
        _co: &Console, 
        _name: *mut i8 , 
        _idx: i32, 
        _options: *mut i8,
    ) -> Result<i32>{
        let mut iotype: c_uchar = 0;
        let mut addr: bindings::resource_size_t = 0;

        /*
         * 受 Qualcomm Technologies QDF2400 E44 勘误影响的系统有一个不同的控制台名称，
         * 所以需要检查该名称。实际的勘误实现发生在探测函数中。
         */
        let name_cstr = unsafe { CStr::from_char_ptr(_name) };
        let name_str = name_cstr.to_str()?;
        if name_str != "qdf2400_e44" && name_str != "pl011" { 
            return Err(ENODEV);  // 如果名称不匹配，返回 -ENODEV
        }

        // 如果解析早期控制台参数失败，返回 -ENODEV
        if unsafe {
            bindings::uart_parse_earlycon(
                _options,
                &mut iotype as *mut _,
                &mut addr as *mut _,
                _options as *mut *mut _,
            ) != 0
        } {
            return Err(ENODEV); // 如果解析早期控制台参数失败，返回 -ENODEV
        }

        if u32::from(iotype) != UPIO_MEM && u32::from(iotype) != UPIO_MEM32 {
            return Err(ENODEV); // 如果 IO 类型不匹配，返回 -ENODEV
        }

        /* 尝试匹配命令行中指定的端口 */
        for i in 0..UART_NR {
            if unsafe { PORTS[i] }.is_none() {
                continue; // 如果端口为空，继续下一次循环
            }

            let port = &unsafe { &PORTS[i] }.unwrap(); // 获取 UART 端口
            let port_ptr = unsafe { &mut *port.as_ptr() };

            if port_ptr.mapbase != addr {
                continue; // 如果地址不匹配，继续下一次循环
            }

            let co_ptr = unsafe { &mut *_co.as_ptr() };
            co_ptr.index = i as i16; // 设置控制台索引
            port_ptr.cons = co_ptr; // 将控制台关联到端口
            return Self::pl011_console_setup(_co, _options); // 调用控制台设置函数
        }

        pr_info!("console_match ok");
        Err(ENODEV)  // 如果没有匹配成功，返回 -ENODEV
    }

    fn console_device(_co: &Console, _index: *mut i8) -> *mut bindings::tty_driver{
        pr_info!("pl011 console_device ok");
        unsafe { bindings::uart_console_device(_co.as_ptr(), _index as *mut i32) }
    }
}

pub(crate) static VENDOR_DATA: VendorData = VendorData {
    ifls:    UART011_IFLS_RX4_8 | UART011_IFLS_TX4_8,
    fr_busy: UART01X_FR_BUSY,
    fr_dsr:  UART01X_FR_DSR,
    fr_cts:  UART01X_FR_CTS,
    fr_ri:   UART011_FR_RI,
    inv_fr:  0,
    access_32b:           false,
    oversampling:         false,
    dma_threshold:        false,
    cts_event_workaround: false,
    always_enabled:       false,
    fixfixed_options:     false,
};

#[derive(Clone, Copy)]
struct PL011Data {
    im: u32,
    old_status: u32,
    fifosize: u32,
    fixed_baud:u32,
    type_ : [i8; 12],
    rs485_tx_started: bool,
    rs485_tx_drain_interval: u32,
    vendor: VendorData,
}

struct PL011Resources {
    base: IoMem<UART_SIZE>,
    parent_irq: u32,
}

type PL011Registrations = PortRegistration<PL011Device>;
type PL011DeviceData = device::Data<PL011Registrations, PL011Resources, PL011Data>;

struct AmbaDeviceData{
    dev_data: Pin<UniqueArc<PL011DeviceData>>
}

impl driver::DeviceRemoval for AmbaDeviceData {
    fn device_remove(&self) {
        pr_info!("********* PL011 data released *********\n");
    }
}

// Linux Raw id table
define_amba_id_table! {MY_AMDA_ID_TABLE, (), [
    ({id: 0x00041011, mask: 0x000fffff}, None),
]}

module_amba_id_table!(UART_MOD_TABLE, MY_AMDA_ID_TABLE);
struct PL011Driver;

impl PL011Driver {
    fn pl011_probe_dt_alias(index: i32, dev: device::Device) -> i32 {
        let mut seen_dev_with_alias = false;
        let mut seen_dev_without_alias = false;
        if bindings::CONFIG_OF == 0 {
            return index;
        }
        let raw_dev = unsafe { *dev.raw_device() };
        let np = raw_dev.of_node;
        if !np.is_null() {
            return index;
        }
        let mut ret = unsafe { bindings::of_alias_get_id(np, "serial".as_ptr() as *const _) };
        if ret < 0 {
            seen_dev_without_alias = true;
            ret = index;
        } else {
            seen_dev_with_alias = true;
            if ret >= UART_NR as i32 || unsafe { !PORTS[ret as usize].is_none() } {
                dev_warn!(&dev, "alias 'serial' has invalid index {}\n", ret);
                return index;
            }
        }

        if seen_dev_with_alias && seen_dev_without_alias {
            dev_warn!(&dev, "aliased and non-aliased serial devices found in device tree. Serial port enumeration may be unpredictable.\n");
        }
        ret
    }
    
}
impl amba::Driver for PL011Driver {
    type Data = Box<AmbaDeviceData>;

    kernel::driver_amba_id_table!(MY_AMDA_ID_TABLE);
    fn probe(adev: &mut amba::Device, _data: Option<&Self::IdInfo>) -> Result<Self::Data> {
        dev_info!(adev,"{} PL061 GPIO chip (probe)\n",adev.name());
        dbg!("********** PL061 GPIO chip (probe) *********\n");

        let dev = device::Device::from_dev(adev);
        let portnr = PL011Device::pl011_find_free_port()?;
        dev_info!(adev,"portnr is {}\n",portnr);
        let _clk = dev.clk_get().unwrap();  // 获得clk
        let fifosize = if adev.revision_get().unwrap() < 3 {16} else {32};
        let iotype = UPIO_MEM as u8;
        let reg_base = adev.take_resource().ok_or(ENXIO)?;
        let reg_mem : IoMem<UART_SIZE>= unsafe { IoMem::try_new(&reg_base)?};
        let mapbase = reg_base.get_offset();
        let membase = reg_mem.get();
        let irq = adev.irq(0).ok_or(ENXIO)?;

        dev_info!(adev,"fifosize is {}\n",fifosize);
        dev_info!(adev,"mapbase is 0x{:x}\n", mapbase);
        dev_info!(adev,"membase is 0x{:p}\n",membase);
        dev_info!(adev,"irq is {}\n",irq);
        let has_sysrq = 1;
        let flags = UPF_BOOT_AUTOCONF;
        let index = Self::pl011_probe_dt_alias(portnr as i32, dev.clone());
        dev_info!(adev,"index is {}\n",index);
        let port =  Box::try_new(UartPort::new().setup(
            membase, 
            mapbase, 
            irq,
            iotype,
            flags, 
            has_sysrq,
            fifosize,
            index as _,
        ))?;

        let rev = adev.revision_get().unwrap();
        let type_:[i8; 12] = [
            'P' as _, 'L' as _, '0' as _, '1' as _, '1' as _, ' ' as _, 
            'r' as _, 'e' as _, 'v' as _, (rev+48) as _, 0, 0,   
        ];
        let pl011_data = PL011Data{
            im: 0,
            old_status: 0,
            fifosize,
            fixed_baud: 0,
            type_,
            rs485_tx_started: false,
            rs485_tx_drain_interval: 0,
            vendor: VENDOR_DATA,
        };

        let pl011_res = PL011Resources{
            base: reg_mem,
            parent_irq: irq,
        };

        let mut pl011_reg = PL011Registrations::new(*port);
        // let mut port_reg = Box::try_new(pl011_reg)?;
        let mut pin = unsafe { Pin::new_unchecked(&mut pl011_reg) };
        PL011Device::pl011_write(0, membase, UART011_IMSC as usize, iotype);
        PL011Device::pl011_write(0xffff, membase, UART011_ICR as usize, iotype);

        if !unsafe { IS_INIT } {
            let ret = unsafe { bindings::uart_register_driver(UART_DRIVER.as_ptr()) };
            if ret < 0 {
                pr_err!("Failed to register uart driver\n");
                return Err(EINVAL);
            }
            unsafe { IS_INIT = true };
        }
        pin.as_mut().register(adev, &UART_DRIVER, Box::try_new(pl011_data)?)?;

        unsafe { PORTS[index as usize] = Some(*port) };

        let dev_data = new_device_data!(pl011_reg, pl011_res, pl011_data, "ttyAMBA")?;
        Ok(Box::try_new(AmbaDeviceData{
            dev_data
        })?)
    }

    fn remove(_data: &Self::Data) {
        dbg!("********* PL061 GPIO chip removed *********\n");
        let data = _data.as_ref().dev_data.deref();
        let mut binding = data.registrations().unwrap();
        let reg = binding.deref_mut();
        let mut pin = unsafe { Pin::new_unchecked(reg) };
        pin.unregister(&UART_DRIVER);
        let uart_port = unsafe { *binding.deref_mut().get_port().as_ptr() };
        for (index, _) in unsafe { PORTS.iter().enumerate() } {
            if index == uart_port.line as usize {
                unsafe { PORTS[index] = None };
                break;
            }
        }
    }
}

module_amba_driver! {
    type: PL011Driver,
    name: "pl011_uart",
    author: "Tang hanwen",
    license: "GPL",
    initcall: "arch",
}

struct PL011Device{}

impl PL011Device {
    /// Find available driver ports sequentially.
    fn pl011_find_free_port() -> Result<usize>{
        for (index, port) in unsafe { PORTS.iter().enumerate() } {
            if let None = port {
                return Ok(index)
            }
        }
        return Err(EBUSY);
    }

    /// pl011 register write
    fn pl011_write(val:u32, membase: *mut u8, reg: usize, iotype: u8) {
        let addr = membase.wrapping_add(reg);
        if iotype == UPIO_MEM32 as u8 {
            unsafe { bindings::writel_relaxed(val as _, addr as _) };
        } else {
            unsafe { bindings::writew_relaxed(val as _, addr as _) };
        }

    }

    /// pl011 register read
    fn pl011_read(membase: *mut u8, reg: usize, iotype: u8) -> u32 {
        let addr = membase.wrapping_add(reg);
        if iotype == UPIO_MEM32 as u8 {
            unsafe { bindings::readl_relaxed(addr as _) as _ }
        } else {
            unsafe { bindings::readw_relaxed(addr as _) as _ }
        }
    }

    fn pl011_tx_char(port: &UartPort, c: u8, from_irq: bool) -> bool {
        let pl011_port = unsafe { &mut *port.as_ptr() };
        if unsafe { bindings::unlikely(!from_irq) } && 
            Self::pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype) != 0 {
                return false;
        }
        Self::pl011_write(c as u32, pl011_port.membase, UART01X_DR as usize, pl011_port.iotype);
        unsafe { bindings::mb() };
        pl011_port.icount.tx += 1;
        return true;
    }

    fn pl011_tx_chars(port: &UartPort, from_irq: bool) -> bool {
        // FIXME
        let mut pl011_port = unsafe { *port.as_ptr() };
        let mut xmit = unsafe{ (*pl011_port.state).xmit };
        let mut count = pl011_port.fifosize >> 1;

        if pl011_port.x_char != 0 {
            if !Self::pl011_tx_char(port, pl011_port.x_char, from_irq) {
                return true;
            }
            pl011_port.x_char = 0;
            count -= 1;
        }
        if uart_circ_empty(&xmit) || 
                unsafe { bindings::uart_tx_stopped(port.as_ptr()) != 0 }{
            Self::stop_tx(port);
            return false;
        }

        loop {
            let cond = unsafe { bindings::likely(from_irq) };
            count -= 1;
            if cond && count+1 == 0 {
                break;
            }
            if cond && count == 0 &&
                Self::pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype) != 0 {
                break;
            }
            // let c = xmit.buf.wrapping_add(xmit.tail as usize) as u8;
            let c = unsafe { *xmit.buf.wrapping_add(xmit.tail as usize) };
            if Self::pl011_tx_char(port, c as u8, from_irq) {
                break;
            }
        
            xmit.tail = (xmit.tail + 1) & (UART_XMIT_SIZE - 1);
            if uart_circ_empty(&xmit) {
                break;
            }
        }

        return true;

    }

    fn pl011_tx_empty(port: &UartPort) -> u32 {
        let pl011_port = unsafe { *port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        let status = Self::pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype);
        if status & (data.vendor.fr_busy|UART01X_FR_TXFF) != 0 {
            bindings::TIOCSER_TEMT
        } else {
            0
        }
    }

    fn pl011_rx_chars(port: &UartPort) {
        let mut pl011_port = unsafe { *port.as_ptr() };
        Self::pl011_fifo_to_tty(port);

        unsafe { bindings::spin_unlock(&mut pl011_port.lock) }
        unsafe { bindings::tty_flip_buffer_push(&mut (*pl011_port.state).port) };
        unsafe { bindings::spin_lock(&mut pl011_port.lock) }
    }


    fn pl011_rs485_tx_start(port: &UartPort) {
        let pl011_port = unsafe { *port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        /* Enable transmitter */
        let mut cr: u32 = Self::pl011_read(pl011_port.membase, UART011_CR as usize, pl011_port.iotype);
        cr |= UART011_CR_TXE;

        /* Disable receiver if half-duplex */
        if pl011_port.rs485.flags & bindings::SER_RS485_RX_DURING_TX == 0 {
            cr &= !UART011_CR_RXE;
        }

        if pl011_port.rs485.flags & bindings::SER_RS485_RTS_ON_SEND != 0 {
            cr |= UART011_CR_RTS;
        } else {
            cr &= !UART011_CR_RTS;
        }

        Self::pl011_write(cr, pl011_port.membase, UART011_CR as usize, pl011_port.iotype);

        if pl011_port.rs485.delay_rts_after_send != 0{
            unsafe { bindings::msleep(pl011_port.rs485.delay_rts_after_send) };
        }

        data.rs485_tx_started = true;
    }

    fn pl011_rs485_tx_stop(port: &UartPort) {
        let pl011_port = unsafe { *port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        let mut cr = Self::pl011_read(pl011_port.membase, UART011_CR as usize, pl011_port.iotype);
        let max_tx_drain_iters = pl011_port.fifosize * 2;
        let mut i = 0;
        while Self::pl011_tx_empty(&port) == 0 {
            if i > max_tx_drain_iters {
                break;
            }
            unsafe { bindings::__udelay(data.rs485_tx_drain_interval as u64); };
            i += 1;
        }

        if pl011_port.rs485.delay_rts_after_send != 0 {
            unsafe { bindings::mdelay(pl011_port.rs485.delay_rts_after_send); };
        }
        cr = Self::pl011_read(pl011_port.membase, UART011_CR as usize, pl011_port.iotype);

        if pl011_port.rs485.flags & SER_RS485_RTS_AFTER_SEND != 0 {
            cr &= !UART011_CR_RTS;
        } else {
            cr |= UART011_CR_RTS;
        }

        cr &= !UART011_CR_TXE;
        cr |= UART011_CR_RXE;

        Self::pl011_write(cr, pl011_port.membase, UART011_CR as usize, pl011_port.iotype);
        data.rs485_tx_started = false;
    }

    fn pl011_start_tx_pio(port: &UartPort) {
        let pl011_port = unsafe { *port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        if Self::pl011_tx_chars(port, false) {
            data.im |= UART011_TXIM;
            Self::pl011_write(data.im, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
        }
    }

    fn pl011_quiesce_irq(port: &UartPort) {
        let pl011_port = unsafe { *port.as_ptr() };
        let mut val = Self::pl011_read(pl011_port.membase, UART011_MIS as usize, pl011_port.iotype);
        Self::pl011_write(val, pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
        val = Self::pl011_read(pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
        Self::pl011_write(val & !UART011_TXIM, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype)
    }

    fn pl011_fifo_to_tty(port: &UartPort) {
        let mut fifotaken = 0;
        let mut sysrq = 0;
        let pl011_port = unsafe { *port.as_ptr() };
        let port_ptr = port.as_ptr();
        while fifotaken != 256 {
            let status = Self::pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype);
            if status & UART01X_FR_RXFE != 0 {
                break;
            }

            let mut ch = Self::pl011_read(pl011_port.membase, !(UART01X_DR|UART011_DR_PE) as usize, pl011_port.iotype);
            let mut flag = TTY_NORMAL;
            (unsafe { *port_ptr }).icount.rx += 1;

            if unsafe { bindings::unlikely((ch & UART_DR_ERROR) != 0) } {
                if ch & UART011_DR_BE != 0 {
                    ch &= !(UART011_DR_FE | UART011_DR_PE);
                    (unsafe { *port_ptr }).icount.brk += 1;
                    if unsafe { bindings::uart_handle_break(port_ptr) != 0 } {
                        continue;
                    }
                } else if ch & UART011_DR_PE != 0 {
                    (unsafe { *port_ptr }).icount.parity += 1;
                } else if ch & UART011_DR_FE != 0 {
                    (unsafe { *port_ptr }).icount.frame += 1;
                } 
                if ch & UART011_DR_FE != 0 {
                    (unsafe { *port_ptr }).icount.overrun += 1;
                }

                ch &= pl011_port.read_status_mask;

                if ch & UART011_DR_BE != 0 {
                    flag = TTY_BREAK;
                } else if ch & UART011_DR_PE != 0 {
                    flag = TTY_PARITY;
                } else if ch & UART011_DR_FE != 0 {
                    flag = TTY_FRAME;
                }
            }

            unsafe { bindings::spin_unlock(&mut (*port_ptr).lock) };
            sysrq = unsafe { bindings::uart_handle_sysrq_char(port_ptr, (ch & 255) as u8) };
            unsafe { bindings::spin_lock(&mut (*port_ptr).lock) };

            if !sysrq != 0 {
                unsafe { bindings::uart_insert_char(port_ptr, ch, UART011_DR_OE, ch as u8, flag) };
            }
            fifotaken += 1;
        }
    }

    fn pl011_modem_status(port: &UartPort) {
        let port_ptr = port.as_ptr();
        let pl011_port = unsafe { *port_ptr };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        let status= Self::pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype) & UART01X_FR_MODEM_ANY;
        let delta = status ^ data.old_status;
        data.old_status = status;
        if !delta != 0 {
            return;
        }

        if delta & UART01X_FR_DCD != 0 {
            unsafe { bindings::uart_handle_dcd_change(port_ptr, status & UART01X_FR_DCD != 0) };
        }

        if delta & data.vendor.fr_dsr != 0 {
            (unsafe { *port_ptr }).icount.dsr += 1;
        }

        if delta & data.vendor.fr_cts != 0 {
            unsafe { bindings::uart_handle_cts_change(port_ptr,
                                             status & data.vendor.fr_cts != 0) }; 
        }
	    unsafe { bindings::wake_up_interruptible(&mut (*pl011_port.state).port.delta_msr_wait) };
    }

    fn pl011_enbale_interrupts(port: &UartPort) {
        let mut pl011_port = unsafe { *port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        let flags: u64 = 0;
        unsafe { bindings::spin_lock_irqsave(&mut pl011_port.lock, flags) };

        Self::pl011_write(UART011_RTIS | UART011_RXIS, pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);

        for i in 0..data.fifosize * 2 {
            if Self::pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype) & UART01X_FR_RXFE != 0 {
                break;
            }
            Self::pl011_read(pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
        }

        Self::pl011_write(data.im, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);

        unsafe { bindings::spin_unlock_irqrestore(&mut pl011_port.lock, flags) };
    }

    fn pl011_disable_interrupts(port: &UartPort) {
        let mut pl011_port = unsafe { *port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        unsafe { bindings::spin_lock_irq(&mut pl011_port.lock) };

        data.im = 0;
        Self::pl011_write(data.im, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
        Self::pl011_write(0xffff, pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);

        unsafe { bindings::spin_unlock_irq(&mut pl011_port.lock) };
    }

    fn pl011_shutdown_channel(port: &UartPort, lcrh: u32) {
        let pl011_port = unsafe { *port.as_ptr() };
        let mut val = Self::pl011_read(pl011_port.membase, lcrh as usize, pl011_port.iotype);
        val &= !(UART01X_LCRH_BRK | UART01X_LCRH_FEN);
        Self::pl011_write(val, pl011_port.membase, lcrh as usize, pl011_port.iotype)
    }

    fn pl011_disable_uart(port: &UartPort) {
        let mut pl011_port = unsafe { *port.as_ptr() };
        let mut cr: u32 = 0;

        unsafe { (*port.as_ptr()).status &= !(UPSTAT_AUTOCTS | UPSTAT_AUTORTS) };
        unsafe { bindings::spin_lock_irq(&mut pl011_port.lock) };
        cr = Self::pl011_read(pl011_port.membase, UART011_CR as usize, pl011_port.iotype);
        cr &= UART011_CR_RTS | UART011_CR_DTR;
        cr |= UART01X_CR_UARTEN | UART011_CR_TXE;
        Self::pl011_write(cr, pl011_port.membase, UART011_CR as usize, pl011_port.iotype);
        unsafe { bindings::spin_unlock_irq(&mut pl011_port.lock) };

        Self::pl011_shutdown_channel(&port, UART011_LCRH as u32);
    }

    fn pl011_allocate_irq(port: &UartPort) -> Result<irq::Registration<PL011Irq>> {
        let pl011_port = unsafe { *port.as_ptr() };
        let data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        Self::pl011_write(data.im, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
        request_irq(pl011_port.irq, unsafe { <Box<UartPort> as ForeignOwnable>::from_foreign(port.as_ptr() as *const c_void) } )
    }
}

#[vtable]
impl UartPortOps for PL011Device {

    #[doc = " User data that will be accessible to all operations"]
    type Data = Box<PL011Data>;

    #[doc = " * @tx_empty:      check if the UART TX FIFO is empty"]
    fn tx_empty(_port: &UartPort) -> u32 {
        let port = unsafe { *_port.as_ptr() };
        let status = Self::pl011_read(port.membase, UART01X_FR as usize, port.iotype);
        if status & UART01X_FR_TXFF != 0 {
            bindings::TIOCSER_TEMT
        } else {
            0
        }
    }

    #[doc = " * @set_mctrl:    set the modem control register"]
    fn set_mctrl(_port: &UartPort, mctrl:u32) {
        let port = unsafe { *_port.as_ptr() };
        let mut cr = Self::pl011_read(port.membase, UART010_CR as usize, port.iotype);

        let mut tiocmbit = |tiocmbit: u32, uartbit: u32| {
            if mctrl & tiocmbit != 0 {
                cr |= uartbit;
            } else {
                cr &= !uartbit;
            }
        };
        tiocmbit(bindings::TIOCM_RTS, UART011_CR_RTS);
        tiocmbit(bindings::TIOCM_DTR, UART011_CR_DTR);
        tiocmbit(bindings::TIOCM_OUT1, UART011_CR_OUT1);
        tiocmbit(bindings::TIOCM_OUT2, UART011_CR_OUT2);
        tiocmbit(bindings::TIOCM_LOOP, UART011_CR_LBE);

        if port.status & UPSTAT_AUTOCTS != 0 {
            tiocmbit(bindings::TIOCM_RTS, UART011_CR_RTSEN);
        }

        Self::pl011_write(cr, port.membase, UART011_CR as usize, port.iotype)
    }

    #[doc = " * @get_mctrl:    get the modem control register"]
    fn get_mctrl(_port: &UartPort) -> u32 {
        let port = unsafe { *_port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        let mut result = 0;
        let status = Self::pl011_read(port.membase, UART01X_FR as usize, port.iotype);

        let mut tiocmbit = |uartbit: u32, tiocmbit: u32| {
            if status & uartbit != 0 {
                result |= tiocmbit;
            }
        };
        tiocmbit(UART01X_FR_DCD, bindings::TIOCM_CAR);
        tiocmbit(data.vendor.fr_dsr, bindings::TIOCM_DSR);
        tiocmbit(data.vendor.fr_cts, bindings::TIOCM_CTS);
        tiocmbit(data.vendor.fr_ri, bindings::TIOCM_RNG);

        return result;
    }

    #[doc = " * @stop_tx:      stop transmitting"]
    fn stop_tx(_port: &UartPort) {
        let port = unsafe { *_port.as_ptr() };
        let data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        }; 
        data.im &= !UART011_TXIM;
        Self::pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype);
    }

    #[doc = " * @start_tx:    start transmitting"]
    fn start_tx(_port: &UartPort) {
        let port = unsafe { *_port.as_ptr() };
        let data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        }; 
        if (port.rs485.flags & bindings::SER_RS485_ENABLED != 0) && 
            !data.rs485_tx_started {
            Self::pl011_rs485_tx_start(_port);
        } 
        Self::pl011_start_tx_pio(_port);
    }

    #[doc = " * @throttle:     stop receiving"]
    fn throttle(&_port: &UartPort) {
        let mut port = unsafe { *_port.as_ptr() };
        let flags: u64 = 0;
        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };
        Self::stop_rx(&_port);
        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };
    }

    #[doc = " * @unthrottle:   start receiving"]
    fn unthrottle(_port: &UartPort) {
        let mut port = unsafe { *_port.as_ptr() };        
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        let flags: u64 = 0;

        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };

        data.im = UART011_RTIM | UART011_RXIM;
        Self::pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype);

        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };
    }

    #[doc = " * @send_xchar:  send a break character"]
    fn send_xchar(_port: &UartPort,ch:i8) {
    }

    #[doc = " * @stop_rx:      stop receiving"]
    fn stop_rx(_port: &UartPort) {
        let port = unsafe { *_port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        data.im &= !(UART011_RXIM | UART011_RTIM | UART011_FEIM | 
                        UART011_PEIM | UART011_BEIM | UART011_OEIM);
        Self::pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype)
    }

    #[doc = " * @start_rx:    start receiving"]
    fn start_rx(_port: &UartPort) {
    }

    #[doc = " * @enable_ms:    enable modem status interrupts"]
    fn enable_ms(_port: &UartPort) {
        let port = unsafe { *_port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        data.im |= UART011_RIMIM | UART011_CTSMIM | UART011_DCDMIM | UART011_DSRMIM;
        Self::pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype)
    }

    #[doc = " * @break_ctl:   set the break control"]
    fn break_ctl(_port: &UartPort, ctl:i32) {
        let mut port = unsafe { *_port.as_ptr() };
        let flags: u64 = 0;

        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };
        let mut lcr_h = Self::pl011_read(port.membase, ST_UART011_LCRH_TX as usize, port.iotype);
        if ctl == -1 {
            lcr_h |= UART01X_LCRH_BRK;
        } else {
            lcr_h &= !UART01X_LCRH_BRK;
        }
        Self::pl011_write(lcr_h, port.membase, UART011_LCRH as usize, port.iotype);
        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };
    }

    #[doc = " * @startup:      start the UART"]
    fn startup(_port: &UartPort) -> i32 {
        let mut port = unsafe { *_port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        let retval = Self::poll_init(_port);
        if retval != 0 {
            // data.clk.disable_unprepare();
            return retval;
        }
        let res = Self::pl011_allocate_irq(&_port);
        if res.is_err() {
            // data.clk.disable_unprepare();
            return 1;
        }
        Self::pl011_write(data.vendor.ifls, port.membase, UART011_IFLS as usize, port.iotype);
        unsafe { bindings::spin_lock_irq(&mut port.lock) };
        let mut cr = Self::pl011_read(port.membase, UART011_CR as usize, port.iotype);
        cr &= UART011_CR_RTS | UART011_CR_DTR;
        cr |= UART01X_CR_UARTEN | UART011_CR_RXE;

        if port.rs485.flags & SER_RS485_ENABLED == 0 {
            cr |= UART011_CR_TXE;
        }
        Self::pl011_write(cr, port.membase, UART011_CR as usize, port.iotype);
        unsafe { bindings::spin_unlock_irq(&mut port.lock) };
        data.old_status = Self::pl011_read(port.membase, UART01X_FR as usize, port.iotype) & UART01X_FR_MODEM_ANY;

        Self::pl011_enbale_interrupts(_port);
        return 0;
        // TODO
    }

    #[doc = " * @shutdown:     shutdown the UART"]
    fn shutdown(_port: &UartPort) {
        let port = unsafe { *_port.as_ptr() };
        let data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        Self::pl011_disable_interrupts(&_port);

        if port.rs485.flags & bindings::SER_RS485_ENABLED != 0 && 
                data.rs485_tx_started {
            Self::pl011_rs485_tx_stop(&_port);
        }

        unsafe { bindings::free_irq(port.irq, _port.as_ptr() as *mut c_void) };

        Self::pl011_disable_uart(&_port);
        unsafe { bindings::pinctrl_pm_select_sleep_state(port.dev) };
        // TODO
    }

    #[doc = " * @flush_buffer: flush the UART buffer"]
    fn flush_buffer(_port: &UartPort) {
        todo!()
    }

    #[doc = " * @set_termios: set the termios structure"]
    fn set_termios(_port: &UartPort,_new: *mut bindings::ktermios,_old: *const bindings::ktermios,) {
        todo!()
    }

    #[doc = " * @set_ldisc:    set the line discipline"]
    fn set_ldisc(_port: &UartPort,_arg2: *mut bindings::ktermios) {
        todo!()
    }

    #[doc = " * @pm:            power management"]
    fn pm(_port: &UartPort,_state:u32,_oldstate:u32) {
        todo!()
    }

    #[doc = " * @type:          get the type of the UART"]
    fn port_type(_port: &UartPort) ->  *const i8 {
        let port = unsafe { *_port.as_ptr() };
        let data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        if port.type_ == PORT_AMBA {
            data.type_.as_ptr()
        } else {
            null()
        }
    }

    #[doc = " * @release_port: release the UART port"]
    fn release_port(uart_port: &UartPort) {
        todo!()
    }

    #[doc = " * @request_port: request the UART port"]
    fn request_port(uart_port: &UartPort) -> i32 {
        todo!()
    }

    #[doc = " * @config_port:  configure the UART port"]
    fn config_port(uart_port: &UartPort,flags:i32) {
        let mut port = unsafe { *uart_port.as_ptr() };
        if flags & UART_CONFIG_TYPE as i32 != 0 {
            port.type_ = PORT_AMBA;
        }
    }

    #[doc = " * @verify_port:  verify the UART port"]
    fn verify_port(uart_port: &UartPort,ser: *mut bindings::serial_struct) -> i32 {
        let port = unsafe { *uart_port.as_ptr() };
        let ser_dref = unsafe { *ser };
        let mut ret: i32 = 0;
        if ser_dref.type_ != PORT_UNKNOWN as i32 && ser_dref.type_ != PORT_AMBA as i32 {
            ret = EINVAL.to_errno();
        }
        if ser_dref.irq < 0 || ser_dref.irq >= NR_IRQS as i32 {
            ret = EINVAL.to_errno();
        }
        if ser_dref.baud_base < 9600 {
            ret = EINVAL.to_errno();
        }
        if port.mapbase != ser_dref.iomem_base as u64 {
            ret = EINVAL.to_errno();
        }
        ret
    }

    #[doc = " * @ioctl:        ioctl handler"]
    fn ioctl(uart_port: &UartPort,arg2:u32,arg3:u64) -> i32 {
        todo!()
    }

    #[doc = " #[cfg(CONFIG_CONSOLE_POLL)]"]
    fn poll_init(uart_port: &UartPort) -> i32 {
        let port = unsafe { *uart_port.as_ptr() };
        let mut data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(port.private_data)
        };
        unsafe { bindings::pinctrl_pm_select_default_state(port.dev) };
        // let ret = data.clk.prepare_enable();
        // if ret.is_err() {
        //     return 0;
        // }
        // port.uartclk = data.clk.get_rate() as u32;
        dbg!("====Serial: uartclk is {}", port.uartclk);
        Self::pl011_write(UART011_OEIS | UART011_BEIS | UART011_PEIS | 
                UART011_FEIS | UART011_RTIS | UART011_RXIS, 
                port.membase, UART011_IMSC as usize, port.iotype);
        
        data.im = Self::pl011_read(port.membase, UART011_IMSC as usize, port.iotype);
        Self::pl011_write(UART011_RTIM | UART011_RXIM, port.membase, UART011_IMSC as usize, port.iotype);

        // TODO: Implement amba_pl011_data
        return 0;
    }

    #[doc = " #[cfg(CONFIG_CONSOLE_POLL)]"]
    fn poll_put_char(uart_port: &UartPort, arg2:u8) {
        let port = unsafe { *uart_port.as_ptr() };
        while Self::pl011_read(port.membase, UART01X_FR as usize, port.iotype) & UART01X_FR_TXFF != 0 {
            unsafe { bindings::cpu_relax() };
        }
        Self::pl011_write(arg2 as u32, port.membase, UART01X_DR as usize, port.iotype);
    }

    #[doc = " #[cfg(CONFIG_CONSOLE_POLL)]"]
    fn poll_get_char(uart_port: &UartPort) -> i32 {
        let port = unsafe { *uart_port.as_ptr() };
        Self::pl011_quiesce_irq(uart_port);
        let status = Self::pl011_read(port.membase, UART01X_FR as usize, port.iotype);
        if status & UART01X_FR_RXFE != 0 {
            return NO_POLL_CHAR as i32;
        }
        return Self::pl011_read(port.membase, UART01X_DR as usize, port.iotype) as i32;
    }
}

struct PL011Irq;

impl PL011Irq {
    fn check_apply_cts_event_workaround(port: &UartPort) {
        let pl011_port = unsafe { *port.as_ptr() };
        let data: Box<PL011Data> = unsafe {
            <Box<PL011Data> as ForeignOwnable>::from_foreign(pl011_port.private_data)
        };
        if !data.vendor.cts_event_workaround {
            return;
        }
        PL011Device::pl011_write(0x00, pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
        PL011Device::pl011_read(pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
        PL011Device::pl011_read(pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
    }
}

impl irq::Handler for PL011Irq {
    type Data = Box<UartPort>;

    fn handle_irq(data: &UartPort) -> irq::Return {
        let mut port = unsafe { *data.as_ptr() };
        let flags: u64 = 0;
        let mut status = AMBA_ISR_PASS_LIMIT;
        let mut pass_counter  = AMBA_ISR_PASS_LIMIT;
        let mut handled = 0;

        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };
        status = PL011Device::pl011_read(port.membase, UART011_RIS as usize, port.iotype) as i32;
        if status != 0 {
            loop {
                Self::check_apply_cts_event_workaround(data);
                PL011Device::pl011_write(status as u32 & !(UART011_TXIS|UART011_RTIS|UART011_RXIS), 
                        port.membase, UART011_ICR as usize, port.iotype);

                if status as u32 & (UART011_RTIS|UART011_RXIS) != 0 {
                    PL011Device::pl011_rx_chars(&data);
                }

                if status as u32 & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS) != 0 {
                    PL011Device::pl011_modem_status(&data);
                }

                if status as u32 & UART011_TXIS != 0 {
                    PL011Device::pl011_tx_chars(&data, true);
                }

                if pass_counter == 0 {
                    pass_counter -= 1;
                    break;
                }

                pass_counter -= 1;
                status = PL011Device::pl011_read(port.membase, UART011_RIS as usize, port.iotype) as i32;
                if status == 0 {
                 break;
                } 
            }
            handled = 1;
        }
        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };

        if handled == 0 {
            irq::Return::None
        } else {
            irq::Return::Handled
        }
    }
}

fn request_irq(irq: u32, data: Box<UartPort>) -> Result<irq::Registration<PL011Irq>> {
    irq::Registration::try_new(irq, data, irq::flags::SHARED, fmt!("uart-pl011"))
}

