//! QoS profile encoding/decoding for liveliness tokens.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use core::fmt::Display;

/// History depth that rmw_zenoh_cpp substitutes when a liveliness token
/// omits the depth (i.e. the depth equals rmw_zenoh's default profile).
/// Keep in sync with `RMW_ZENOH_DEFAULT_HISTORY_DEPTH` in rmw_zenoh's
/// `rmw_zenoh_cpp/src/detail/qos.cpp`.
const RMW_ZENOH_DEFAULT_HISTORY_DEPTH: usize = 42;

/// QoS profile for ROS 2 entities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct QosProfile {
    pub reliability: QosReliability,
    pub durability: QosDurability,
    pub history: QosHistory,
}

impl QosProfile {
    /// Encode QoS to string for liveliness token.
    /// Format matches rmw_zenoh_cpp: [reliability]:[durability]:[history],[depth]:[deadline]:[lifespan]:[liveliness]
    pub fn encode(&self) -> String {
        use alloc::format;
        let default_qos = Self::default();

        // Reliability - empty if default (RMW values: 1=Reliable, 2=BestEffort)
        let reliability = if self.reliability != default_qos.reliability {
            match self.reliability {
                QosReliability::Reliable => "1",
                QosReliability::BestEffort => "2",
            }
        } else {
            ""
        };

        // Durability - empty if default (RMW values: 1=TransientLocal, 2=Volatile)
        let durability = if self.durability != default_qos.durability {
            match self.durability {
                QosDurability::TransientLocal => "1",
                QosDurability::Volatile => "2",
            }
        } else {
            ""
        };

        // History format: <history_kind>,<depth>
        // Only include kind if non-default, always include depth
        let history = match self.history {
            QosHistory::KeepLast(depth) => {
                if self.history != default_qos.history {
                    format!("1,{}", depth)
                } else {
                    format!(",{}", depth)
                }
            }
            QosHistory::KeepAll => "2,".to_string(),
        };

        // Deadline, lifespan, liveliness - use defaults (empty/infinite)
        let deadline = ",";
        let lifespan = ",";
        let liveliness = ",,";

        format!(
            "{}:{}:{}:{}:{}:{}",
            reliability, durability, history, deadline, lifespan, liveliness
        )
    }

    /// Decode QoS from liveliness token string.
    pub fn decode(s: &str) -> Result<Self, QosDecodeError> {
        let fields: alloc::vec::Vec<&str> = s.split(':').collect();
        if fields.len() < 3 {
            return Err(QosDecodeError::InvalidFormat);
        }

        let default_qos = Self::default();

        // Parse reliability (RMW values: 1=Reliable, 2=BestEffort)
        let reliability = match fields[0] {
            "" => default_qos.reliability,
            "1" => QosReliability::Reliable,
            "2" => QosReliability::BestEffort,
            _ => return Err(QosDecodeError::InvalidReliability),
        };

        // Parse durability (RMW values: 1=TransientLocal, 2=Volatile)
        let durability = match fields[1] {
            "" => default_qos.durability,
            "1" => QosDurability::TransientLocal,
            "2" => QosDurability::Volatile,
            _ => return Err(QosDecodeError::InvalidDurability),
        };

        // Parse history: <kind>,<depth>
        //
        // rmw_zenoh_cpp omits every QoS component that equals its default
        // profile, so an endpoint with default history encodes the whole
        // field as `,` (empty kind AND empty depth) — e.g. the
        // `::,:,:,:,,` / `:1:,:,:,:,,` suffixes emitted by ros2_control
        // controller nodes. An empty kind means the default kind
        // (KEEP_LAST) and an empty depth means the emitter's default
        // depth. rmw_zenoh_cpp's own `keyexpr_to_qos` restores its
        // default depth (42, see rmw_zenoh's `qos.cpp`); mirror that here
        // instead of rejecting the token.
        let history_parts: alloc::vec::Vec<&str> = fields[2].split(',').collect();
        if history_parts.len() < 2 {
            return Err(QosDecodeError::InvalidHistory);
        }

        let history = match history_parts[0] {
            // "" = default kind; "0" = RMW SYSTEM_DEFAULT, which
            // rmw_zenoh resolves to its default kind; "1" = KEEP_LAST.
            "" | "0" | "1" => {
                let depth = match history_parts[1] {
                    "" => RMW_ZENOH_DEFAULT_HISTORY_DEPTH,
                    depth => depth
                        .parse::<usize>()
                        .map_err(|_| QosDecodeError::InvalidHistory)?,
                };
                QosHistory::KeepLast(depth)
            }
            "2" => QosHistory::KeepAll,
            _ => return Err(QosDecodeError::InvalidHistory),
        };

        Ok(QosProfile {
            reliability,
            durability,
            history,
        })
    }
}

/// QoS reliability policy.
///
/// ROS 2 default: Reliable
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum QosReliability {
    BestEffort = 0,
    #[default]
    Reliable = 1,
}

/// QoS durability policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum QosDurability {
    #[default]
    Volatile = 0,
    TransientLocal = 1,
}

/// QoS history policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum QosHistory {
    KeepLast(usize),
    KeepAll,
}

impl Default for QosHistory {
    fn default() -> Self {
        QosHistory::KeepLast(10)
    }
}

impl QosHistory {
    pub fn from_depth(depth: usize) -> Self {
        QosHistory::KeepLast(depth)
    }

    pub fn depth(&self) -> usize {
        match self {
            QosHistory::KeepLast(d) => *d,
            QosHistory::KeepAll => 0,
        }
    }
}

/// QoS decode errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosDecodeError {
    InvalidFormat,
    InvalidReliability,
    InvalidDurability,
    InvalidHistory,
}

impl Display for QosDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            QosDecodeError::InvalidFormat => write!(f, "Invalid QoS format"),
            QosDecodeError::InvalidReliability => write!(f, "Invalid reliability value"),
            QosDecodeError::InvalidDurability => write!(f, "Invalid durability value"),
            QosDecodeError::InvalidHistory => write!(f, "Invalid history value"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// rmw_zenoh_cpp omits every QoS component equal to its default profile.
    /// An endpoint with an entirely-default profile therefore encodes as
    /// `::,:,:,:,,` (empty history kind AND depth). Emitted in the wild by
    /// ros2_control 6.x controller nodes; must decode, not error.
    #[test]
    fn decode_fully_default_rmw_zenoh_qos() {
        let qos = QosProfile::decode("::,:,:,:,,").expect("default-omitted QoS must decode");
        assert_eq!(qos.reliability, QosReliability::Reliable);
        assert_eq!(qos.durability, QosDurability::Volatile);
        assert_eq!(
            qos.history,
            QosHistory::KeepLast(RMW_ZENOH_DEFAULT_HISTORY_DEPTH)
        );
    }

    /// Same as above with an explicit non-default durability
    /// (`:1:,:,:,:,,` — transient-local, everything else default).
    #[test]
    fn decode_transient_local_with_default_history() {
        let qos = QosProfile::decode(":1:,:,:,:,,").expect("QoS with omitted history must decode");
        assert_eq!(qos.reliability, QosReliability::Reliable);
        assert_eq!(qos.durability, QosDurability::TransientLocal);
        assert_eq!(
            qos.history,
            QosHistory::KeepLast(RMW_ZENOH_DEFAULT_HISTORY_DEPTH)
        );
    }

    /// Explicit depth with omitted (default) history kind: `::,100:...`.
    #[test]
    fn decode_explicit_depth_default_kind() {
        let qos = QosProfile::decode("::,100:,:,:,,").expect("explicit depth must decode");
        assert_eq!(qos.history, QosHistory::KeepLast(100));
    }

    /// Keep-all history (`2,`): depth carries no meaning and may be empty.
    #[test]
    fn decode_keep_all() {
        let qos = QosProfile::decode("::2,:,:,:,,").expect("keep-all must decode");
        assert_eq!(qos.history, QosHistory::KeepAll);
    }

    /// Round-trip through our own encoder still works.
    #[test]
    fn encode_decode_roundtrip() {
        let qos = QosProfile {
            reliability: QosReliability::BestEffort,
            durability: QosDurability::TransientLocal,
            history: QosHistory::KeepLast(7),
        };
        let decoded = QosProfile::decode(&qos.encode()).expect("roundtrip");
        assert_eq!(decoded, qos);
    }

    /// Garbage in the history kind still errors.
    #[test]
    fn decode_invalid_history_kind_rejected() {
        assert_eq!(
            QosProfile::decode("::x,:,:,:,,"),
            Err(QosDecodeError::InvalidHistory)
        );
    }
}
