// SPDX-License-Identifier: GPL-2.0

//! Driver for AMBA serial ports (PL011).
//!
//! Based on the C driver written by ARM Ltd/Deep Blue Solutions Ltd.

use core::{any::Any, clone::Clone, convert::AsRef, ops::{Deref, DerefMut}, ptr::null};

use kernel::{
    amba, bindings, c_str, clk::{self, Clk}, define_amba_id_table, device::{self, Data, Device}, driver::DeviceRemoval, driver_amba_id_table, error::{code::*, Result}, io_mem::IoMem, irq, module_amba_driver, module_amba_id_table, prelude::*, print, serial:: {
        pl011_config::*, uart_console::{flags, Console, ConsoleOps}, uart_driver::UartDriver, uart_port::{uart_circ_empty, PortRegistration, UartPort, UartPortOps}
    }, sync::{Arc, ArcBorrow}, types::ForeignOwnable
};

const UART_SIZE: usize = 0x200;
const UPIO_MEM: u32 = 2;
const UPIO_MEM32: u32 = 3;

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

/// This uart_driver static's struct
pub(crate) static UART_DRIVER: UartDriver = UartDriver::new(
    &THIS_MODULE, 
    DRIVER_NAME, 
    DEV_NAME,
    &AMBA_CONSOLE
    ).with_config(
        AMBA_MAJOR, 
        AMBA_MINOR, 
        UART_NR as _,
    );

/// This is Struct of pl011_console
struct Pl011Console;
/// Implement supported `Pl011Console`'s operations here.
#[vtable]
impl ConsoleOps for Pl011Console {
    type Data = ();

    fn console_write(_co: &Console, _s: *const i8, _count: u32) {
        pr_info!("console_write ok");
    }

    fn console_read(_co: &Console, _s: *mut i8, _count: u32) -> Result<i32>{
        pr_info!("console_read ok");
        Ok(0)
    }

    fn console_match(
        _co: &Console, 
        _name: *mut i8 , 
        _idx: i32, 
        _options: *mut i8,
    ) -> Result<i32>{
        pr_info!("console_match ok");
        Ok(0)
    }

    fn console_device(_co: &Console, _index: *mut i8) -> *mut bindings::tty_driver{
        pr_info!("console_device ok");
        todo!()
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

struct PL011Data {
    reg_offset: u16,
    im: u32,
    old_status: u32,
    clk: Clk,
    fifosize: u32,
    fixed_baud:u32,
    type_ : [i8; 12],
    rs485_tx_started: bool,
}

struct PL011Resources {
    base: IoMem<UART_SIZE>,
    parent_irq: u32,
}

type PL011Registrations = PortRegistration<PL011Device>;
type PL011DeviceData = device::Data<PL011Registrations, PL011Resources, PL011Data>;

// Linux Raw id table
define_amba_id_table! {MY_AMDA_ID_TABLE, (), [
    ({id: 0x00041011, mask: 0x000fffff}, None),
]}

struct PL011Device;
impl amba::Driver for PL011Device {
    // type Data = Arc<PL011DeviceData>;

    kernel::driver_amba_id_table!(MY_AMDA_ID_TABLE);
    fn probe(adev: &mut amba::Device, _data: Option<&Self::IdInfo>) -> Result {
        dev_info!(adev,"{} PL061 GPIO chip (probe)\n",adev.name());
        dbg!("********** PL061 GPIO chip (probe) *********\n");

        let dev = device::Device::from_dev(adev);

        let portnr = pl011_find_free_port()?;
        dev_info!(adev,"portnr is {}\n",portnr);
        let clk = dev.clk_get().unwrap();  // 获得clk
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
        let port =  UartPort::new().setup(
                membase, 
                mapbase, 
                irq,
                iotype,
                flags, 
                has_sysrq,
                fifosize,
             portnr as _,
            );     

        dbg!("********* PL061 GPIO chip registered *********\n");
        Ok(())
    }

    fn remove(_data: &Self::Data) {
        dbg!("********* PL061 GPIO chip removed *********\n");
        // unsafe { bindings::uart_remove_one_port(&UART_DRIVER, port) };
    }
}

module_amba_driver! {
    type: PL011Device,
    name: "pl011_uart",
    author: "Tang hanwen",
    license: "GPL",
    initcall: "arch",
}

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
    let mut pl011_port = unsafe { *port.as_ptr() };
    if unsafe { bindings::unlikely(!from_irq) } && 
        pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype) != 0 {
            return false;
    }
    pl011_write(c as u32, pl011_port.membase, UART01X_DR as usize, pl011_port.iotype);
    unsafe { bindings::mb() };
    pl011_port.icount.tx += 1;
    return true;
}

fn pl011_tx_chars(port: &UartPort,  data: &mut Box<PL011Data>, from_irq: bool) -> bool {
    let mut pl011_port = unsafe { *port.as_ptr() };
    let mut xmit = unsafe{ (*pl011_port.state).xmit };
    let mut count = pl011_port.fifosize >> 1;

    if pl011_port.x_char != 0 {
        if !pl011_tx_char(port, pl011_port.x_char, from_irq) {
            return true;
        }
        pl011_port.x_char = 0;
        count -= 1;
    }
    // TODO: Implement uart_tx_stopped
    if uart_circ_empty(&xmit) {
        PL011Device::stop_tx(port, data);
        return false;
    }

    loop {
        let cond = unsafe { bindings::likely(from_irq) };
        count -= 1;
        if cond && count+1 == 0 {
            break;
        }
        if cond && count == 0 &&
            pl011_read(pl011_port.membase, UART01X_FR as usize, pl011_port.iotype) != 0 {
            break;
        }
        // let c = xmit.buf.wrapping_add(xmit.tail as usize) as u8;
        let c = unsafe { *xmit.buf.wrapping_add(xmit.tail as usize) };
        if pl011_tx_char(port, c as u8, from_irq) {
            break;
        }
    
        xmit.tail = (xmit.tail + 1) & (UART_XMIT_SIZE - 1);
        if uart_circ_empty(&xmit) {
            break;
        }
    }

    return true;

}

fn pl011_rs485_tx_start(port: &UartPort, data: &mut Box<PL011Data>) {
    let pl011_port = unsafe { *port.as_ptr() };

	/* Enable transmitter */
    let mut cr: u32 = pl011_read(pl011_port.membase, UART011_CR as usize, pl011_port.iotype);
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

    pl011_write(cr, pl011_port.membase, UART011_CR as usize, pl011_port.iotype);

    if pl011_port.rs485.delay_rts_after_send != 0{
        unsafe { bindings::msleep(pl011_port.rs485.delay_rts_after_send) };
    }

    data.rs485_tx_started = true;
}

fn pl011_start_tx_pio(port: &UartPort, data: &mut Box<PL011Data>) {
    let pl011_port = unsafe { *port.as_ptr() };
    if pl011_tx_chars(port, data, false) {
        data.im |= UART011_TXIM;
        pl011_write(data.im, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
    }
}

fn pl011_quiesce_irq(port: &UartPort) {
    let pl011_port = unsafe { *port.as_ptr() };
    let mut val = pl011_read(pl011_port.membase, UART011_MIS as usize, pl011_port.iotype);
    pl011_write(val, pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
    val = pl011_read(pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype);
    pl011_write(val & !UART011_TXIM, pl011_port.membase, UART011_IMSC as usize, pl011_port.iotype)
}


#[vtable]
impl UartPortOps for PL011Device {

    #[doc = " User data that will be accessible to all operations"]
    type Data = Box<PL011Data>;

    #[doc = " * @tx_empty:      check if the UART TX FIFO is empty"]
    fn tx_empty(_port: &UartPort) -> u32 {
        let port = unsafe { *_port.as_ptr() };
        let status = pl011_read(port.membase, UART01X_FR as usize, port.iotype);
        if status & UART01X_FR_TXFF != 0 {
            bindings::TIOCSER_TEMT
        } else {
            0
        }
    }

    #[doc = " * @set_mctrl:    set the modem control register"]
    fn set_mctrl(_port: &UartPort, mctrl:u32) {
        let port = unsafe { *_port.as_ptr() };
        let mut cr = pl011_read(port.membase, UART010_CR as usize, port.iotype);

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

        pl011_write(cr, port.membase, UART011_CR as usize, port.iotype)
    }

    #[doc = " * @get_mctrl:    get the modem control register"]
    fn get_mctrl(_port: &UartPort) -> u32 {
        let port = unsafe { *_port.as_ptr() };
        let mut result = 0;
        let status = pl011_read(port.membase, UART01X_FR as usize, port.iotype);

        let mut tiocmbit = |uartbit: u32, tiocmbit: u32| {
            if status & uartbit != 0 {
                result |= tiocmbit;
            }
        };
        tiocmbit(UART01X_FR_DCD, bindings::TIOCM_CAR);
        tiocmbit(VENDOR_DATA.fr_dsr, bindings::TIOCM_DSR);
        tiocmbit(VENDOR_DATA.fr_cts, bindings::TIOCM_CTS);
        tiocmbit(VENDOR_DATA.fr_ri, bindings::TIOCM_RNG);

        return result;
    }

    #[doc = " * @stop_tx:      stop transmitting"]
    fn stop_tx(_port: &UartPort, data: &mut Self::Data) {
        let port = unsafe { *_port.as_ptr() };
        data.im &= !UART011_TXIM;
        pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype);
    }

    #[doc = " * @start_tx:    start transmitting"]
    fn start_tx(_port: &UartPort, data: &mut Self::Data) {
        let port = unsafe { *_port.as_ptr() };

        if (port.rs485.flags & bindings::SER_RS485_ENABLED != 0) && 
            !data.rs485_tx_started {
            pl011_rs485_tx_start(_port, data);
        } 
        pl011_start_tx_pio(_port, data);
    }

    #[doc = " * @throttle:     stop receiving"]
    fn throttle(&_port: &UartPort, data: &mut Self::Data) {
        let mut port = unsafe { *_port.as_ptr() };
        let flags: u64 = 0;
        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };
        // PL011PortOps::stop_rx(&_port, &mut data);
        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };
    }

    #[doc = " * @unthrottle:   start receiving"]
    fn unthrottle(_port: &UartPort, data: &mut Self::Data) {
        let mut port = unsafe { *_port.as_ptr() };
        let flags: u64 = 0;

        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };

        data.im = UART011_RTIM | UART011_RXIM;
        pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype);

        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };
    }

    #[doc = " * @send_xchar:  send a break character"]
    fn send_xchar(_port: &UartPort,ch:i8) {
    }

    #[doc = " * @stop_rx:      stop receiving"]
    fn stop_rx(_port: &UartPort, data: &mut Self::Data) {
        let port = unsafe { *_port.as_ptr() };
        data.im &= !(UART011_RXIM | UART011_RTIM | UART011_FEIM | 
                        UART011_PEIM | UART011_BEIM | UART011_OEIM);
        pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype)
    }

    #[doc = " * @start_rx:    start receiving"]
    fn start_rx(_port: &UartPort, data: &mut Self::Data) {
    }

    #[doc = " * @enable_ms:    enable modem status interrupts"]
    fn enable_ms(_port: &UartPort, data: &mut Self::Data) {
        let port = unsafe { *_port.as_ptr() };
        data.im |= UART011_RIMIM | UART011_CTSMIM | UART011_DCDMIM | UART011_DSRMIM;
        pl011_write(data.im, port.membase, UART011_IMSC as usize, port.iotype)
    }

    #[doc = " * @break_ctl:   set the break control"]
    fn break_ctl(_port: &UartPort, ctl:i32) {
        let mut port = unsafe { *_port.as_ptr() };
        let flags: u64 = 0;

        unsafe { bindings::spin_lock_irqsave(&mut port.lock, flags) };
        let mut lcr_h = pl011_read(port.membase, ST_UART011_LCRH_TX as usize, port.iotype);
        if ctl == -1 {
            lcr_h |= UART01X_LCRH_BRK;
        } else {
            lcr_h &= !UART01X_LCRH_BRK;
        }
        pl011_write(lcr_h, port.membase, UART011_LCRH as usize, port.iotype);
        unsafe { bindings::spin_unlock_irqrestore(&mut port.lock, flags) };
    }

    #[doc = " * @startup:      start the UART"]
    fn startup(_port: &UartPort, data: &mut Self::Data) -> i32 {
        let port = unsafe { *_port.as_ptr() };
        let mut retval = PL011Device::poll_init(_port, data);
        if retval != 0 {
            data.clk.disable_unprepare();
        }
        todo!()
    }

    #[doc = " * @shutdown:     shutdown the UART"]
    fn shutdown(_port: &UartPort) {
        todo!()
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
    fn port_type(_port: &UartPort, data: &mut Self::Data) ->  *const i8 {
        let port = unsafe { *_port.as_ptr() };
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
    fn poll_init(uart_port: &UartPort, data: &mut Self::Data) -> i32 {
        let mut port = unsafe { *uart_port.as_ptr() };
        unsafe { bindings::pinctrl_pm_select_default_state(port.dev) };
        let ret = data.clk.prepare_enable();
        if ret.is_err() {
            return 0;
        }
        port.uartclk = data.clk.get_rate() as u32;
        dbg!("====Serial: uartclk is {}", port.uartclk);
        pl011_write(UART011_OEIS | UART011_BEIS | UART011_PEIS | 
                UART011_FEIS | UART011_RTIS | UART011_RXIS, 
                port.membase, UART011_IMSC as usize, port.iotype);
        
        data.im = pl011_read(port.membase, UART011_IMSC as usize, port.iotype);
        pl011_write(UART011_RTIM | UART011_RXIM, port.membase, UART011_IMSC as usize, port.iotype);

        // TODO: Implement amba_pl011_data

        return 0;
    }

    #[doc = " #[cfg(CONFIG_CONSOLE_POLL)]"]
    fn poll_put_char(uart_port: &UartPort, arg2:u8) {
        let port = unsafe { *uart_port.as_ptr() };
        while pl011_read(port.membase, UART01X_FR as usize, port.iotype) & UART01X_FR_TXFF != 0 {
            unsafe { bindings::cpu_relax() };
        }
        pl011_write(arg2 as u32, port.membase, UART01X_DR as usize, port.iotype);
    }

    #[doc = " #[cfg(CONFIG_CONSOLE_POLL)]"]
    fn poll_get_char(uart_port: &UartPort) -> i32 {
        let port = unsafe { *uart_port.as_ptr() };
        pl011_quiesce_irq(uart_port);
        let status = pl011_read(port.membase, UART01X_FR as usize, port.iotype);
        if status & UART01X_FR_RXFE != 0 {
            return NO_POLL_CHAR as i32;
        }
        return pl011_read(port.membase, UART01X_DR as usize, port.iotype) as i32;
    }
}

struct IrqPrivateData {
    pl011_hw_ops: Arc<PL011Device>,
    port: Box<UartPort>,
    data: Box<PL011DeviceData>
}

struct PL011Irq;

impl irq::Handler for PL011Irq {
    type Data = Box<IrqPrivateData>;

    fn handle_irq(data: &IrqPrivateData) -> irq::Return {
        let mut port_ptr= data.port.as_ptr();
        let port = unsafe { *port_ptr };
        let flags: u64 = 0;
        let mut status = AMBA_ISR_PASS_LIMIT;
        let pass_counter  = AMBA_ISR_PASS_LIMIT;
        let mut handled = 0;

        unsafe { bindings::spin_lock_irqsave(&mut (*port_ptr).lock, flags) };
        status = pl011_read(port.membase, UART011_RIS as usize, port.iotype) as i32;
        if status != 0 {
            loop {
                check_apply_cts_event_workaround(data.port.as_ref());
                pl011_write(status as u32 & !(UART011_TXIS|UART011_RTIS|UART011_RXIS), 
                        port.membase, UART011_ICR as usize, port.iotype);
                status = pl011_read(port.membase, UART011_RIS as usize, port.iotype) as i32;
                if status == 0 {
                 break;
                } 
            }
            handled = 1;
        }
        unsafe { bindings::spin_unlock_irqrestore(&mut (*port_ptr).lock, flags) };

        if handled == 0 {
            irq::Return::None
        } else {
            irq::Return::Handled
        }
    }
}

fn check_apply_cts_event_workaround(port: &UartPort) {
    let pl011_port = unsafe { *port.as_ptr() };
    if !VENDOR_DATA.cts_event_workaround {
        return;
    }
    pl011_write(0x00, pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
    pl011_read(pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
    pl011_read(pl011_port.membase, UART011_ICR as usize, pl011_port.iotype);
}