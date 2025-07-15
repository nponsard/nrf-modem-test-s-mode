#![no_std]
#![no_main]
use embassy_nrf::pac::uicr::vals::{Hfxocnt, Hfxosrc};
use embassy_nrf::pac::NVMC;
use futures::stream::{self, StreamExt};
use tinyrlibc::*;

use cortex_m::peripheral::NVIC;
use defmt::{debug, expect, info};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{
    interrupt, pac,
    pac::{NVMC_S, UICR_S},
};
use embassy_time::Timer;
use futures_core::Stream;
use nrf_modem::{ConnectionPreference, MemoryLayout};
use nrf_modem::{GnssData, SystemMode};
use {defmt_rtt as _, panic_probe as _};

extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
}

fn uicr_hfxo_workaround() {
    let uicr = embassy_nrf::pac::UICR_S;
    let hfxocnt = uicr.hfxocnt().read().hfxocnt().to_bits();
    let hfxosrc = uicr.hfxosrc().read().hfxosrc().to_bits();

    if hfxocnt != 255 && hfxosrc != 1 {
        return;
    }

    let irq_disabled = cortex_m::register::primask::read().is_inactive();
    if !irq_disabled {
        cortex_m::interrupt::disable();
    }
    cortex_m::asm::dsb();
    while !NVMC_S.ready().read().ready() {}

    NVMC_S
        .config()
        .write(|w| w.set_wen(pac::nvmc::vals::Wen::WEN));
    while !NVMC_S.ready().read().ready() {}

    if hfxosrc == 1 {
        UICR_S.hfxosrc().write(|w| w.set_hfxosrc(Hfxosrc::TCXO));
        cortex_m::asm::dsb();
        while !NVMC_S.ready().read().ready() {}
    }

    if hfxocnt == 255 {
        UICR_S.hfxocnt().write(|w| w.set_hfxocnt(Hfxocnt(32)));
        cortex_m::asm::dsb();
        while !NVMC_S.ready().read().ready() {}
    }

    NVMC_S
        .config()
        .write(|w| w.set_wen(pac::nvmc::vals::Wen::REN));
    while !NVMC_S.ready().read().ready() {}

    if !irq_disabled {
        unsafe {
            cortex_m::interrupt::enable();
        }
    }

    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initializing embassy_nrf has to come first because it assumes POWER and CLOCK at the secure address
    let embassy_peripherals = embassy_nrf::init(Default::default());
    let mut led = Output::new(embassy_peripherals.P0_00, Level::Low, OutputDrive::Standard);

    let clock = embassy_nrf::pac::CLOCK_S;
    // clock.tasks_hfclkstart().write_value(1);
    let stat = clock.hfclkstat().read();
    defmt::info!("HFCLK Source: {}", stat.src().to_bits());
    defmt::info!("HFCLK State: {}", stat.state());

    let uicr = embassy_nrf::pac::UICR_S;
    let hfxocnt = uicr.hfxocnt().read().hfxocnt().to_bits();
    let hfxosrc = uicr.hfxosrc().read().hfxosrc().to_bits();

    defmt::info!("HFXO Count: {}", hfxocnt);
    defmt::info!("HFXO Source: {}", hfxosrc);

    uicr_hfxo_workaround();

    let hfxocnt = uicr.hfxocnt().read().hfxocnt().to_bits();
    let hfxosrc = uicr.hfxosrc().read().hfxosrc().to_bits();

    defmt::info!("HFXO Count: {}", hfxocnt);
    defmt::info!("HFXO Source: {}", hfxosrc);
    fn configure_modem_non_secure() -> u32 {
        // The RAM memory space is divided into 32 regions of 8 KiB.
        // Set IPC RAM to nonsecure
        const SPU_REGION_SIZE: u32 = 0x2000; // 8kb
        const RAM_START: u32 = 0x2000_0000; // 256kb
        let ipc_start: u32 = unsafe { &__start_ipc as *const u8 } as u32;
        let ipc_reg_offset = (ipc_start - RAM_START) / SPU_REGION_SIZE;
        let ipc_reg_count =
            (unsafe { &__end_ipc as *const u8 } as u32 - ipc_start) / SPU_REGION_SIZE;
        let spu = embassy_nrf::pac::SPU;
        let range = ipc_reg_offset..(ipc_reg_offset + ipc_reg_count);
        debug!("marking region as non secure: {}", range);
        for i in range {
            spu.ramregion(i as usize).perm().write(|w| {
                w.set_execute(true);
                w.set_write(true);
                w.set_read(true);
                w.set_secattr(false);
                w.set_lock(false);
            })
        }

        // Set regulator access registers to nonsecure
        spu.periphid(4).perm().write(|w| w.set_secattr(false));
        // Set clock and power access registers to nonsecure
        spu.periphid(5).perm().write(|w| w.set_secattr(false));
        // Set IPC access register to nonsecure
        spu.periphid(42).perm().write(|w| w.set_secattr(false));
        ipc_start
    }
    let ipc_start = configure_modem_non_secure();
    // Interrupt Handler for LTE related hardware. Defer straight to the library.
    #[interrupt]
    #[allow(non_snake_case)]
    fn IPC() {
        nrf_modem::ipc_irq_handler();
    }

    let mut cp = cortex_m::Peripherals::take().expect("Failed to take Cortex-M peripherals");

    // Enable the modem interrupts
    unsafe {
        NVIC::unmask(pac::Interrupt::IPC);
        cp.NVIC.set_priority(pac::Interrupt::IPC, 0 << 5);
    }

    nrf_modem::init_with_custom_layout(
        SystemMode {
            lte_support: true,
            lte_psm_support: true,
            nbiot_support: true,
            gnss_support: true,
            preference: ConnectionPreference::None,
        },
        MemoryLayout {
            base_address: ipc_start,
            tx_area_size: 0x2000,
            rx_area_size: 0x2000,
            trace_area_size: 0x1000,
        },
    )
    .await
    .unwrap();

    let response = nrf_modem::send_at::<64>("AT+CGMI").await.unwrap();
    defmt::info!("Modem Manufacturer: {}", response.as_str());
    let gnss = nrf_modem::Gnss::new().await.unwrap();
    defmt::info!("GNSS initialized");
    let mut stream = gnss.start_single_fix(
        nrf_modem::GnssConfig {
            elevation_threshold_angle: 1,
            use_case: nrf_modem::GnssUsecase {
                low_accuracy: true,
                scheduled_downloads_disable: false,
            },
            nmea_mask: nrf_modem::NmeaMask {
                gga: true,
                gll: true,
                gsa: true,
                gsv: true,
                rmc: true,
            },
            timing_source: nrf_modem::GnssTimingSource::Tcxo,
            power_mode: nrf_modem::GnssPowerSaveMode::DutyCyclingPerformance,
        },
        1000,
    );
    defmt::info!("GNSS stream started");
    let mut stream = match stream {
        Ok(s) => s,
        Err(e) => {
            defmt::error!("Failed to start GNSS: {:?}", e);
            return;
        }
    };

    defmt::info!("GNSS stream is ready");
    while let Some(value) = stream.next().await {
        defmt::debug!("GNSS event");
        if let Err(e) = value {
            defmt::error!("GNSS Error: {:?}", e);
            continue;
        }
        if let Ok(evt) = value {
            match evt {
                GnssData::Agps(agps) => {
                    defmt::info!("GNSS AGPS: {:?}", agps.data_flags);
                }
                GnssData::Nmea(nmea) => {
                    defmt::info!("GNSS NMEA: {}", nmea.as_str());
                }
                GnssData::PositionVelocityTime(pos) => {
                    defmt::info!(
                        "GNSS Position: {},{},{}",
                        pos.latitude,
                        pos.longitude,
                        pos.altitude
                    );
                }
            }
        }
    }
}
