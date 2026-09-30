//! ROS 2-style YAML parameter file loading.
//!
//! Supports the standard ROS 2 parameter file format:
//!
//! ```yaml
//! /**:
//!   ros__parameters:
//!     my_param: 42
//!     another: "hello"
//!
//! /my_node:
//!   ros__parameters:
//!     node_specific: true
//!
//! /my_ns/my_node:
//!   ros__parameters:
//!     nested_param: 3.14
//! ```
//!
//! Node selectors are matched against the node's fully-qualified name
//! (`/{namespace}/{node_name}`, or `/{node_name}` in the root namespace).
//! A `*` segment matches one name segment and `**` any number, so `/**`
//! matches every node. Selectors may omit the leading `/`, and namespace
//! levels may be nested keys. Nested parameter mappings flatten to dotted
//! names, and JSON is accepted as it is valid YAML.

use std::collections::HashMap;
use std::path::Path;

use serde_yaml::{Mapping, Value};

use super::types::ParameterValue;

const ROS_PARAMETERS_KEY: &str = "ros__parameters";

/// The parameters a parameter file lists under one node selector.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeParameters {
    /// The node selector, e.g. `/**`, `/my_node`, or `/my_ns/my_node`.
    ///
    /// Selectors written without a leading `/` are normalized to start with
    /// one, and nested namespace keys are joined with `/`.
    pub selector: String,
    /// Parameter names and values, in file order. Nested mappings are
    /// flattened into dotted names (`a: {b: 1}` becomes `a.b`).
    pub parameters: Vec<(String, ParameterValue)>,
}

/// Load parameter overrides from a YAML file for the given node.
///
/// Returns a map of parameter name → value containing only the parameters
/// applicable to the specified node (by its fully-qualified name).
///
/// # Format
///
/// ```yaml
/// /**:
///   ros__parameters:
///     global_param: 1
///
/// /my_node:
///   ros__parameters:
///     local_param: "hello"
/// ```
pub fn load_parameter_file(
    path: &Path,
    node_fqn: &str,
) -> Result<HashMap<String, ParameterValue>, String> {
    Ok(overrides_for_node(&parse_parameter_file(path)?, node_fqn))
}

/// Parse a YAML string and extract parameter overrides for the given node.
pub fn load_parameter_string(
    yaml: &str,
    node_fqn: &str,
) -> Result<HashMap<String, ParameterValue>, String> {
    Ok(overrides_for_node(&parse_parameter_string(yaml)?, node_fqn))
}

/// Read and parse a parameter file into its per-selector entries, without
/// matching them against a node.
pub fn parse_parameter_file(path: &Path) -> Result<Vec<NodeParameters>, String> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read parameter file {:?}: {}", path, e))?;
    parse_parameter_string(&content)
        .map_err(|e| format!("Invalid parameter file {:?}: {}", path, e))
}

/// Parse a parameter YAML string into its per-selector entries, in file
/// order, without matching them against a node.
///
/// The whole document is validated: a malformed entry is an error even if
/// it would not match the node that eventually reads the file.
pub fn parse_parameter_string(yaml: &str) -> Result<Vec<NodeParameters>, String> {
    let doc: Value =
        serde_yaml::from_str(yaml).map_err(|e| format!("Failed to parse YAML: {}", e))?;

    let mapping = doc
        .as_mapping()
        .ok_or_else(|| "YAML root must be a mapping".to_string())?;

    let mut result = Vec::new();
    collect_node_entries(mapping, "", &mut result)?;
    Ok(result)
}

/// Merge the entries of a parsed parameter file that match `node_fqn`.
///
/// Entries apply in file order, so a later matching entry overrides an
/// earlier one.
pub fn overrides_for_node(
    entries: &[NodeParameters],
    node_fqn: &str,
) -> HashMap<String, ParameterValue> {
    let mut result = HashMap::new();
    for entry in entries
        .iter()
        .filter(|entry| matches_node(&entry.selector, node_fqn))
    {
        for (name, value) in &entry.parameters {
            result.insert(name.clone(), value.clone());
        }
    }
    result
}

/// Parse a single parameter value the way a value in a parameter file is
/// parsed, as used by `-p name:=value` on the command line.
///
/// `"5"` is an integer, `"5.0"` a double, `"true"` a bool, `"[1, 2]"` an
/// integer array, and `"'5'"` (quoted) a string.
pub fn parse_parameter_value(text: &str) -> Result<ParameterValue, String> {
    let value: Value = serde_yaml::from_str(text)
        .map_err(|e| format!("Failed to parse value '{}': {}", text, e))?;
    if value.is_mapping() {
        return Err(format!(
            "value '{}' is a mapping; parameter values must be scalars or sequences",
            text
        ));
    }
    yaml_value_to_parameter(&value).map_err(|e| format!("value '{}': {}", text, e))
}

/// Walk the node-selector levels of a parameter file.
///
/// A key whose value holds `ros__parameters` names a node; any other key is
/// a namespace level, joined with `/` (`ns: {node: {ros__parameters: ...}}`
/// selects `/ns/node`).
fn collect_node_entries(
    mapping: &Mapping,
    prefix: &str,
    result: &mut Vec<NodeParameters>,
) -> Result<(), String> {
    for (key, node_val) in mapping {
        let key = key
            .as_str()
            .ok_or_else(|| "YAML keys must be strings".to_string())?;
        let selector = join_selector(prefix, key);

        let node_map = node_val
            .as_mapping()
            .ok_or_else(|| format!("Value for '{}' must be a mapping", selector))?;

        for (inner_key, inner_val) in node_map {
            let inner_key = inner_key
                .as_str()
                .ok_or_else(|| "YAML keys must be strings".to_string())?;
            if inner_key == ROS_PARAMETERS_KEY {
                let params_map = inner_val.as_mapping().ok_or_else(|| {
                    format!(
                        "'{}' of '{}' must be a mapping",
                        ROS_PARAMETERS_KEY, selector
                    )
                })?;
                let mut parameters = Vec::new();
                flatten_parameters(params_map, "", &mut parameters)?;
                result.push(NodeParameters {
                    selector: selector.clone(),
                    parameters,
                });
            } else {
                let mut nested = Mapping::new();
                nested.insert(Value::String(inner_key.to_string()), inner_val.clone());
                collect_node_entries(&nested, &selector, result)?;
            }
        }
    }
    Ok(())
}

fn join_selector(prefix: &str, key: &str) -> String {
    let key = key.trim_matches('/');
    if prefix.is_empty() {
        format!("/{}", key)
    } else {
        format!("{}/{}", prefix, key)
    }
}

/// Flatten nested parameter mappings into dotted names, as ROS does.
fn flatten_parameters(
    mapping: &Mapping,
    prefix: &str,
    result: &mut Vec<(String, ParameterValue)>,
) -> Result<(), String> {
    for (pname, pval) in mapping {
        let name = pname
            .as_str()
            .ok_or_else(|| "Parameter names must be strings".to_string())?;
        let name = if prefix.is_empty() {
            name.to_string()
        } else {
            format!("{}.{}", prefix, name)
        };

        match pval {
            Value::Mapping(nested) => flatten_parameters(nested, &name, result)?,
            _ => {
                let value = yaml_value_to_parameter(pval)
                    .map_err(|e| format!("parameter '{}': {}", name, e))?;
                result.push((name, value));
            }
        }
    }
    Ok(())
}

/// Check whether a node selector matches the given fully-qualified node name.
///
/// Selectors are matched segment by segment:
/// - `*` matches exactly one name segment
/// - `**` matches zero or more name segments
/// - any other segment must match literally
///
/// So `/**` matches every node, `/my_ns/**` every node under `/my_ns`,
/// `/my_ns/*` the nodes directly in `/my_ns`, and `/my_node` only the node
/// with that fully-qualified name.
pub(crate) fn matches_node(selector: &str, node_fqn: &str) -> bool {
    let selector: Vec<&str> = selector.split('/').filter(|s| !s.is_empty()).collect();
    let fqn: Vec<&str> = node_fqn.split('/').filter(|s| !s.is_empty()).collect();
    matches_segments(&selector, &fqn)
}

fn matches_segments(selector: &[&str], fqn: &[&str]) -> bool {
    match selector.split_first() {
        None => fqn.is_empty(),
        Some((&"**", rest)) => (0..=fqn.len()).any(|skip| matches_segments(rest, &fqn[skip..])),
        Some((&segment, rest)) => match fqn.split_first() {
            Some((&name, fqn_rest)) => {
                (segment == "*" || segment == name) && matches_segments(rest, fqn_rest)
            }
            None => false,
        },
    }
}

/// Convert a YAML value to a ParameterValue.
///
/// Type inference rules (matching rcl's YAML parameter parser):
/// - Integer YAML values → Integer
/// - Float YAML values → Double
/// - Boolean YAML values → Bool
/// - String YAML values → String
/// - Sequence of integers → IntegerArray
/// - Sequence of floats → DoubleArray
/// - Sequence of bools → BoolArray
/// - Sequence of strings → StringArray
///
/// Sequences must be homogeneous. Byte arrays cannot be written in YAML, as
/// in ROS.
fn yaml_value_to_parameter(val: &Value) -> Result<ParameterValue, String> {
    match val {
        Value::Bool(b) => Ok(ParameterValue::Bool(*b)),
        Value::Number(n) => n
            .as_i64()
            .map(ParameterValue::Integer)
            .or_else(|| {
                if n.is_u64() {
                    None
                } else {
                    n.as_f64().map(ParameterValue::Double)
                }
            })
            .ok_or_else(|| format!("integer {} does not fit in i64", n)),
        Value::String(s) => Ok(ParameterValue::String(s.clone())),
        Value::Sequence(seq) => infer_sequence_type(seq),
        Value::Null => Ok(ParameterValue::NotSet),
        Value::Mapping(_) => Err("nested mapping is not a parameter value".to_string()),
        Value::Tagged(tagged) => Err(format!("unsupported YAML tag {}", tagged.tag)),
    }
}

fn infer_sequence_type(seq: &[Value]) -> Result<ParameterValue, String> {
    let Some(first) = seq.first() else {
        // Empty sequence — default to StringArray
        return Ok(ParameterValue::StringArray(vec![]));
    };

    let mixed = || "sequence elements must all have the same type".to_string();
    match first {
        Value::Bool(_) => seq
            .iter()
            .map(|v| v.as_bool())
            .collect::<Option<Vec<_>>>()
            .map(ParameterValue::BoolArray)
            .ok_or_else(mixed),
        Value::Number(n) if n.is_i64() => seq
            .iter()
            .map(|v| v.as_i64())
            .collect::<Option<Vec<_>>>()
            .map(ParameterValue::IntegerArray)
            .ok_or_else(mixed),
        Value::Number(n) if n.is_f64() => seq
            .iter()
            .map(|v| match v {
                Value::Number(n) if n.is_f64() => n.as_f64(),
                _ => None,
            })
            .collect::<Option<Vec<_>>>()
            .map(ParameterValue::DoubleArray)
            .ok_or_else(mixed),
        Value::String(_) => seq
            .iter()
            .map(|v| v.as_str().map(str::to_string))
            .collect::<Option<Vec<_>>>()
            .map(ParameterValue::StringArray)
            .ok_or_else(mixed),
        _ => Err("sequence elements must be bools, integers, doubles, or strings".to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const YAML_SAMPLE: &str = r#"
/**:
  ros__parameters:
    global_int: 42
    global_str: "hello"
    global_bool: true
    global_float: 2.5

/my_node:
  ros__parameters:
    node_specific: 99
    override_me: "original"

/other_node:
  ros__parameters:
    not_mine: 1
"#;

    #[test]
    fn test_wildcard_match() {
        let params = load_parameter_string(YAML_SAMPLE, "/my_node").unwrap();
        // Gets both wildcard and node-specific
        assert_eq!(params["global_int"], ParameterValue::Integer(42));
        assert_eq!(
            params["global_str"],
            ParameterValue::String("hello".to_string())
        );
        assert_eq!(params["global_bool"], ParameterValue::Bool(true));
        assert_eq!(params["global_float"], ParameterValue::Double(2.5));
        assert_eq!(params["node_specific"], ParameterValue::Integer(99));
    }

    #[test]
    fn test_no_other_node_params() {
        let params = load_parameter_string(YAML_SAMPLE, "/my_node").unwrap();
        assert!(!params.contains_key("not_mine"));
    }

    #[test]
    fn test_exact_node_only() {
        let params = load_parameter_string(YAML_SAMPLE, "/other_node").unwrap();
        assert!(params.contains_key("not_mine"));
        assert!(params.contains_key("global_int")); // wildcard still applies
        assert!(!params.contains_key("node_specific"));
    }

    #[test]
    fn test_arrays() {
        let yaml = r#"
/**:
  ros__parameters:
    int_list: [1000, 2000, 3000]
    float_list: [1.0, 2.0, 3.0]
    str_list: ["a", "b"]
    bool_list: [true, false, true]
"#;
        let params = load_parameter_string(yaml, "/any_node").unwrap();
        assert_eq!(
            params["int_list"],
            ParameterValue::IntegerArray(vec![1000, 2000, 3000])
        );
        assert_eq!(
            params["float_list"],
            ParameterValue::DoubleArray(vec![1.0, 2.0, 3.0])
        );
        assert_eq!(
            params["str_list"],
            ParameterValue::StringArray(vec!["a".to_string(), "b".to_string()])
        );
        assert_eq!(
            params["bool_list"],
            ParameterValue::BoolArray(vec![true, false, true])
        );
    }

    #[test]
    fn test_namespace_selector() {
        let yaml = r#"
/my_ns/**:
  ros__parameters:
    ns_param: 1
/other_ns/**:
  ros__parameters:
    other_param: 2
"#;
        let params = load_parameter_string(yaml, "/my_ns/my_node").unwrap();
        assert!(params.contains_key("ns_param"));
        assert!(!params.contains_key("other_param"));
    }

    #[test]
    fn test_matches_node() {
        assert!(matches_node("/**", "/any/node"));
        assert!(matches_node("/**", "/node"));
        assert!(matches_node("/my_ns/**", "/my_ns/node"));
        assert!(!matches_node("/my_ns/**", "/other_ns/node"));
        assert!(matches_node("/my_node", "/my_node"));
        assert!(!matches_node("/my_node", "/other_node"));
        assert!(!matches_node("/my_ns/**", "/my_nsx/node"));
        assert!(matches_node("/my_ns/**", "/my_ns/a/b"));
        assert!(matches_node("/my_ns/*", "/my_ns/node"));
        assert!(!matches_node("/my_ns/*", "/my_ns/a/b"));
        assert!(matches_node("/**/node", "/a/b/node"));
        assert!(matches_node("/**/node", "/node"));
        assert!(matches_node("/*", "/node"));
        assert!(!matches_node("/*", "/ns/node"));
        // A bare node name selects the root-namespace node only.
        assert!(!matches_node("/node", "/ns/node"));
    }

    #[test]
    fn test_nested_parameters_flatten_to_dotted_names() {
        let yaml = r#"
/**:
  ros__parameters:
    db:
      url: "sqlite:cell.db"
      pool:
        size: 4
    top: 1
"#;
        let params = load_parameter_string(yaml, "/node").unwrap();
        assert_eq!(
            params["db.url"],
            ParameterValue::String("sqlite:cell.db".into())
        );
        assert_eq!(params["db.pool.size"], ParameterValue::Integer(4));
        assert_eq!(params["top"], ParameterValue::Integer(1));
        assert_eq!(params.len(), 3);
    }

    #[test]
    fn test_nested_namespace_keys_and_relative_selectors() {
        let yaml = r#"
my_ns:
  my_node:
    ros__parameters:
      nested_ns: 1
relative_node:
  ros__parameters:
    relative: 2
"#;
        let entries = parse_parameter_string(yaml).unwrap();
        let selectors: Vec<&str> = entries.iter().map(|e| e.selector.as_str()).collect();
        assert_eq!(selectors, ["/my_ns/my_node", "/relative_node"]);
        let params = load_parameter_string(yaml, "/my_ns/my_node").unwrap();
        assert_eq!(params["nested_ns"], ParameterValue::Integer(1));
        let params = load_parameter_string(yaml, "/relative_node").unwrap();
        assert_eq!(params["relative"], ParameterValue::Integer(2));
    }

    #[test]
    fn test_later_entries_win_in_file_order() {
        let yaml = r#"
/my_node:
  ros__parameters:
    p: 1
/**:
  ros__parameters:
    p: 2
"#;
        let params = load_parameter_string(yaml, "/my_node").unwrap();
        assert_eq!(params["p"], ParameterValue::Integer(2));
    }

    #[test]
    fn test_small_integer_sequences_are_integer_arrays() {
        let yaml = "/**:\n  ros__parameters:\n    ids: [1, 2, 3]\n";
        let params = load_parameter_string(yaml, "/node").unwrap();
        assert_eq!(params["ids"], ParameterValue::IntegerArray(vec![1, 2, 3]));
    }

    #[test]
    fn test_unsupported_values_are_errors() {
        for value in [
            "[1, a]",
            "[1, 2.5]",
            "[[1], [2]]",
            "[{a: 1}]",
            "18446744073709551615",
        ] {
            let yaml = format!("/other:\n  ros__parameters:\n    bad: {}\n", value);
            assert!(
                parse_parameter_string(&yaml).is_err(),
                "{} should be rejected",
                value
            );
        }
    }

    #[test]
    fn test_parse_parameter_value() {
        assert_eq!(
            parse_parameter_value("42").unwrap(),
            ParameterValue::Integer(42)
        );
        assert_eq!(
            parse_parameter_value("4.0").unwrap(),
            ParameterValue::Double(4.0)
        );
        assert_eq!(
            parse_parameter_value("'4.0'").unwrap(),
            ParameterValue::String("4.0".into())
        );
        assert_eq!(parse_parameter_value("~").unwrap(), ParameterValue::NotSet);
        assert!(parse_parameter_value("{a: 1}").is_err());
    }
}
