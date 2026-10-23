#![allow(dead_code)]

use pat_core::MachineDef;

pub const TRUNNION_AC: &str = include_str!("../../../../machines/trunnion-ac.toml");
pub const HEAD_TABLE_BC: &str = include_str!("../../../../machines/head-table-bc.toml");
pub const HEAD_HEAD_CA: &str = include_str!("../../../../machines/head-head-ca.toml");
pub const VMC_4TH_A: &str = include_str!("../../../../machines/vmc-4th-a.toml");
pub const VMC_3AXIS: &str = include_str!("../../../../machines/vmc-3axis.toml");

pub fn machine(src: &str) -> MachineDef {
    MachineDef::from_toml(src).expect("example machine is valid")
}

pub fn all_machines() -> Vec<MachineDef> {
    [
        TRUNNION_AC,
        HEAD_TABLE_BC,
        HEAD_HEAD_CA,
        VMC_4TH_A,
        VMC_3AXIS,
    ]
    .iter()
    .map(|s| machine(s))
    .collect()
}

/// Small deterministic PRNG so tests are reproducible without extra crates.
pub struct Lcg(pub u64);

impl Lcg {
    pub fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
}
