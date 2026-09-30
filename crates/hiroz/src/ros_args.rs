//! ROS 2 command-line arguments (`--ros-args ... --`).
//!
//! rclcpp and rclpy nodes take their parameter overrides from the process
//! command line:
//!
//! ```text
//! my_node --ros-args --params-file params.yaml -p max_speed:=2.5 -- --my-flag
//! ```
//!
//! [`RosArgs`] parses that argv the way rcl does and hands the overrides to a
//! node via [`ZNodeBuilder::with_ros_args`](crate::node::ZNodeBuilder::with_ros_args):
//!
//! ```rust,ignore
//! use hiroz::{Builder, ros_args::RosArgs};
//!
//! let ros_args = RosArgs::from_env()?;
//! let node = ctx.create_node("my_node").with_ros_args(&ros_args).build()?;
//! // Everything that was not a ROS argument, program name first.
//! let cli = MyCli::parse_from(ros_args.remaining_args());
//! ```
//!
//! # Supported arguments
//!
//! Inside a `--ros-args` section (which runs until `--` or the end of argv,
//! and may appear more than once):
//!
//! | Argument | Effect |
//! |----------|--------|
//! | `--params-file <path>` | Load parameter overrides from a ROS 2 parameter YAML file |
//! | `-p <name>:=<value>`, `--param <name>:=<value>` | Override one parameter for every node |
//! | `-p <node>:<name>:=<value>` | Override one parameter for the node matching `<node>` |
//!
//! Files are read and validated when the argv is parsed. All sources apply
//! in argv order, so a later `--params-file` or `-p` overrides an earlier one.
//!
//! Any other argument inside a `--ros-args` section is an error, as it is in
//! rclcpp (`UnknownROSArgsError`). That includes ROS arguments hiroz does not
//! implement yet, such as `-r`, `--log-level`, or `--enclave`.

use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

use crate::parameter::{
    ParameterValue,
    yaml::{self, NodeParameters},
};

const ROS_ARGS_FLAG: &str = "--ros-args";
const ROS_ARGS_END: &str = "--";
const PARAMS_FILE_FLAG: &str = "--params-file";
const PARAM_FLAG_SHORT: &str = "-p";
const PARAM_FLAG_LONG: &str = "--param";
const ALL_NODES_SELECTOR: &str = "/**";

/// An error in a ROS 2 command line.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum RosArgsError {
    /// A flag that takes a value was the last argument of its `--ros-args`
    /// section.
    #[error("ROS argument '{flag}' is missing its value")]
    MissingValue { flag: String },
    /// A `-p` rule is not of the form `[node:]name:=value`.
    #[error("invalid parameter rule '{rule}': {reason}")]
    InvalidParamRule { rule: String, reason: String },
    /// A `--params-file` could not be read or parsed.
    #[error("invalid parameter file {path:?}: {reason}")]
    ParamsFile { path: PathBuf, reason: String },
    /// Arguments inside a `--ros-args` section that are not supported ROS
    /// arguments.
    #[error("unknown ROS arguments: {}", .0.join(" "))]
    UnknownArguments(Vec<String>),
}

/// One parameter override source, in argv order.
#[derive(Debug, Clone, PartialEq)]
enum ParameterSource {
    File {
        path: PathBuf,
        entries: Vec<NodeParameters>,
    },
    Rule {
        selector: String,
        name: String,
        value: ParameterValue,
    },
}

/// A parsed ROS 2 command line: the parameter overrides from its
/// `--ros-args` sections and the arguments that were not ROS arguments.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RosArgs {
    sources: Vec<ParameterSource>,
    remaining: Vec<String>,
}

impl RosArgs {
    /// Parse a full argv, program name included (as `std::env::args()`
    /// yields it).
    ///
    /// Parameter files are read and validated here, so a missing or
    /// malformed file fails at startup rather than when a node is built.
    pub fn parse<I, S>(args: I) -> Result<Self, RosArgsError>
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        let mut parsed = Self::default();
        let mut unknown = Vec::new();
        let mut in_ros_args = false;
        let mut args = args.into_iter().map(Into::into);

        while let Some(arg) = args.next() {
            if arg == ROS_ARGS_FLAG {
                in_ros_args = true;
                continue;
            }
            if !in_ros_args {
                parsed.remaining.push(arg);
                continue;
            }
            match arg.as_str() {
                ROS_ARGS_END => in_ros_args = false,
                PARAMS_FILE_FLAG => {
                    let path = PathBuf::from(next_value(&mut args, &arg)?);
                    let entries = yaml::parse_parameter_file(&path).map_err(|reason| {
                        RosArgsError::ParamsFile {
                            path: path.clone(),
                            reason,
                        }
                    })?;
                    parsed.sources.push(ParameterSource::File { path, entries });
                }
                PARAM_FLAG_SHORT | PARAM_FLAG_LONG => {
                    let rule = next_value(&mut args, &arg)?;
                    parsed.sources.push(parse_param_rule(&rule)?);
                }
                _ => unknown.push(arg),
            }
        }

        if unknown.is_empty() {
            Ok(parsed)
        } else {
            Err(RosArgsError::UnknownArguments(unknown))
        }
    }

    /// Parse this process's command line (`std::env::args()`).
    pub fn from_env() -> Result<Self, RosArgsError> {
        Self::parse(std::env::args())
    }

    /// The arguments outside any `--ros-args` section, in order, with the
    /// program name first when it was passed to [`RosArgs::parse`].
    ///
    /// The `--ros-args` and closing `--` markers are not included.
    pub fn remaining_args(&self) -> &[String] {
        &self.remaining
    }

    /// The `--params-file` paths, in argv order.
    pub fn params_files(&self) -> Vec<&Path> {
        self.sources
            .iter()
            .filter_map(|source| match source {
                ParameterSource::File { path, .. } => Some(path.as_path()),
                ParameterSource::Rule { .. } => None,
            })
            .collect()
    }

    /// The parameter overrides that apply to the node with the given
    /// fully-qualified name (`/my_node`, `/my_ns/my_node`).
    ///
    /// Every source applies in argv order and, within a file, in file order;
    /// a later value for the same name wins.
    pub fn parameter_overrides(&self, node_fqn: &str) -> HashMap<String, ParameterValue> {
        let mut result = HashMap::new();
        for source in &self.sources {
            match source {
                ParameterSource::File { entries, .. } => {
                    result.extend(yaml::overrides_for_node(entries, node_fqn));
                }
                ParameterSource::Rule {
                    selector,
                    name,
                    value,
                } => {
                    if yaml::matches_node(selector, node_fqn) {
                        result.insert(name.clone(), value.clone());
                    }
                }
            }
        }
        result
    }
}

fn next_value(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, RosArgsError> {
    match args.next() {
        Some(value) if value != ROS_ARGS_END && value != ROS_ARGS_FLAG => Ok(value),
        _ => Err(RosArgsError::MissingValue {
            flag: flag.to_string(),
        }),
    }
}

/// Parse `name:=value` or `node:name:=value`.
fn parse_param_rule(rule: &str) -> Result<ParameterSource, RosArgsError> {
    let invalid = |reason: &str| RosArgsError::InvalidParamRule {
        rule: rule.to_string(),
        reason: reason.to_string(),
    };

    let (lhs, value_text) = rule
        .split_once(":=")
        .ok_or_else(|| invalid("expected 'name:=value'"))?;
    let (selector, name) = match lhs.split_once(':') {
        Some((node, name)) => {
            if node.is_empty() {
                return Err(invalid("empty node name"));
            }
            let selector = if node.starts_with('/') {
                node.to_string()
            } else {
                format!("/{}", node)
            };
            (selector, name)
        }
        None => (ALL_NODES_SELECTOR.to_string(), lhs),
    };
    if name.is_empty() {
        return Err(invalid("empty parameter name"));
    }
    if value_text.is_empty() {
        return Err(invalid("empty value"));
    }
    let value = yaml::parse_parameter_value(value_text).map_err(|reason| invalid(&reason))?;

    Ok(ParameterSource::Rule {
        selector,
        name: name.to_string(),
        value,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_file(dir: &Path, name: &str, content: &str) -> PathBuf {
        let path = dir.join(name);
        std::fs::write(&path, content).expect("write params file");
        path
    }

    fn temp_dir(test: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("hiroz-ros-args-{}-{}", test, std::process::id()));
        std::fs::create_dir_all(&dir).expect("create temp dir");
        dir
    }

    #[test]
    fn no_ros_args_keeps_everything() {
        let args = RosArgs::parse(["prog", "--flag", "value", "--"]).unwrap();
        assert_eq!(args.remaining_args(), ["prog", "--flag", "value", "--"]);
        assert!(args.parameter_overrides("/node").is_empty());
    }

    #[test]
    fn ros_args_section_ends_at_double_dash() {
        let args =
            RosArgs::parse(["prog", "--ros-args", "-p", "a:=1", "--", "--flag", "-p"]).unwrap();
        assert_eq!(args.remaining_args(), ["prog", "--flag", "-p"]);
        assert_eq!(
            args.parameter_overrides("/node")["a"],
            ParameterValue::Integer(1)
        );
    }

    #[test]
    fn several_ros_args_sections() {
        let args = RosArgs::parse([
            "prog",
            "--ros-args",
            "-p",
            "a:=1",
            "--",
            "x",
            "--ros-args",
            "--param",
            "b:=two",
        ])
        .unwrap();
        assert_eq!(args.remaining_args(), ["prog", "x"]);
        let overrides = args.parameter_overrides("/node");
        assert_eq!(overrides["a"], ParameterValue::Integer(1));
        assert_eq!(overrides["b"], ParameterValue::String("two".into()));
    }

    #[test]
    fn param_values_follow_yaml_scalar_rules() {
        let args = RosArgs::parse([
            "prog",
            "--ros-args",
            "-p",
            "int:=42",
            "-p",
            "neg:=-7",
            "-p",
            "double:=2.5",
            "-p",
            "exp:=1e3",
            "-p",
            "bool:=true",
            "-p",
            "Bool:=False",
            "-p",
            "str:=hello",
            "-p",
            "quoted:='42'",
            "-p",
            "dquoted:=\"true\"",
            "-p",
            "ints:=[1, 2, 3]",
            "-p",
            "doubles:=[1.0, 2.5]",
            "-p",
            "strs:=[a, 'b']",
            "-p",
            "bools:=[true, false]",
            "-p",
            "url:=tcp/localhost:7447",
        ])
        .unwrap();
        let o = args.parameter_overrides("/node");
        assert_eq!(o["int"], ParameterValue::Integer(42));
        assert_eq!(o["neg"], ParameterValue::Integer(-7));
        assert_eq!(o["double"], ParameterValue::Double(2.5));
        assert_eq!(o["exp"], ParameterValue::Double(1000.0));
        assert_eq!(o["bool"], ParameterValue::Bool(true));
        assert_eq!(o["Bool"], ParameterValue::Bool(false));
        assert_eq!(o["str"], ParameterValue::String("hello".into()));
        assert_eq!(o["quoted"], ParameterValue::String("42".into()));
        assert_eq!(o["dquoted"], ParameterValue::String("true".into()));
        assert_eq!(o["ints"], ParameterValue::IntegerArray(vec![1, 2, 3]));
        assert_eq!(o["doubles"], ParameterValue::DoubleArray(vec![1.0, 2.5]));
        assert_eq!(
            o["strs"],
            ParameterValue::StringArray(vec!["a".into(), "b".into()])
        );
        assert_eq!(o["bools"], ParameterValue::BoolArray(vec![true, false]));
        assert_eq!(
            o["url"],
            ParameterValue::String("tcp/localhost:7447".into())
        );
    }

    #[test]
    fn later_param_wins() {
        let args = RosArgs::parse(["prog", "--ros-args", "-p", "a:=1", "-p", "a:=2"]).unwrap();
        assert_eq!(
            args.parameter_overrides("/node")["a"],
            ParameterValue::Integer(2)
        );
    }

    #[test]
    fn node_scoped_param_rule() {
        let args = RosArgs::parse([
            "prog",
            "--ros-args",
            "-p",
            "talker:rate:=10",
            "-p",
            "/ns/listener:rate:=20",
        ])
        .unwrap();
        assert_eq!(
            args.parameter_overrides("/talker")["rate"],
            ParameterValue::Integer(10)
        );
        assert_eq!(
            args.parameter_overrides("/ns/listener")["rate"],
            ParameterValue::Integer(20)
        );
        assert!(args.parameter_overrides("/other").is_empty());
        // `talker:` names the root-namespace node only, as in rcl.
        assert!(args.parameter_overrides("/ns/talker").is_empty());
    }

    #[test]
    fn params_files_apply_in_order_with_params() {
        let dir = temp_dir("order");
        let base = write_file(
            &dir,
            "base.yaml",
            "/**:\n  ros__parameters:\n    a: 1\n    b: 1\n    c: 1\n",
        );
        let layer = write_file(
            &dir,
            "layer.yaml",
            "/node:\n  ros__parameters:\n    b: 2\n    c: 2\n",
        );
        let args = RosArgs::parse([
            "prog".to_string(),
            "--ros-args".to_string(),
            "--params-file".to_string(),
            base.display().to_string(),
            "-p".to_string(),
            "c:=3".to_string(),
            "--params-file".to_string(),
            layer.display().to_string(),
        ])
        .unwrap();

        assert_eq!(args.params_files(), [base.as_path(), layer.as_path()]);
        let o = args.parameter_overrides("/node");
        assert_eq!(o["a"], ParameterValue::Integer(1));
        assert_eq!(o["b"], ParameterValue::Integer(2));
        // The later file overrides the earlier -p.
        assert_eq!(o["c"], ParameterValue::Integer(2));
        // A node the second file does not select keeps the first file's values.
        let other = args.parameter_overrides("/other");
        assert_eq!(other["c"], ParameterValue::Integer(3));
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn params_file_node_selectors() {
        let dir = temp_dir("selectors");
        let file = write_file(
            &dir,
            "sel.json",
            r#"{"/**": {"ros__parameters": {"all": 1}},
                "/backend": {"ros__parameters": {"by_name": 2}},
                "/cell/backend": {"ros__parameters": {"by_fqn": 3}}}"#,
        );
        let args = RosArgs::parse([
            "prog".to_string(),
            "--ros-args".to_string(),
            "--params-file".to_string(),
            file.display().to_string(),
        ])
        .unwrap();

        let root = args.parameter_overrides("/backend");
        assert_eq!(root.len(), 2);
        assert_eq!(root["all"], ParameterValue::Integer(1));
        assert_eq!(root["by_name"], ParameterValue::Integer(2));

        let namespaced = args.parameter_overrides("/cell/backend");
        assert_eq!(namespaced.len(), 2);
        assert_eq!(namespaced["all"], ParameterValue::Integer(1));
        assert_eq!(namespaced["by_fqn"], ParameterValue::Integer(3));
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn params_file_nested_json_flattens() {
        let dir = temp_dir("nested");
        let file = write_file(
            &dir,
            "backend.yaml",
            r#"{"/**":{"ros__parameters":{"db":{"url":"sqlite:x","pool":{"size":4}},"debug":false}}}"#,
        );
        let args = RosArgs::parse([
            "prog".to_string(),
            "--ros-args".to_string(),
            "--params-file".to_string(),
            file.display().to_string(),
        ])
        .unwrap();
        let o = args.parameter_overrides("/backend");
        assert_eq!(o["db.url"], ParameterValue::String("sqlite:x".into()));
        assert_eq!(o["db.pool.size"], ParameterValue::Integer(4));
        assert_eq!(o["debug"], ParameterValue::Bool(false));
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn missing_params_file_is_an_error() {
        let err = RosArgs::parse(["prog", "--ros-args", "--params-file", "/nonexistent.yaml"])
            .unwrap_err();
        assert!(matches!(err, RosArgsError::ParamsFile { .. }), "{err}");
    }

    #[test]
    fn malformed_params_file_is_an_error() {
        let dir = temp_dir("malformed");
        let file = write_file(
            &dir,
            "bad.yaml",
            "/other:\n  ros__parameters:\n    mixed: [1, a]\n",
        );
        let err = RosArgs::parse([
            "prog".to_string(),
            "--ros-args".to_string(),
            "--params-file".to_string(),
            file.display().to_string(),
        ])
        .unwrap_err();
        assert!(matches!(err, RosArgsError::ParamsFile { .. }), "{err}");
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn missing_flag_values_are_errors() {
        for argv in [
            vec!["prog", "--ros-args", "-p"],
            vec!["prog", "--ros-args", "--param", "--"],
            vec!["prog", "--ros-args", "--params-file"],
        ] {
            let err = RosArgs::parse(argv.clone()).unwrap_err();
            assert!(
                matches!(err, RosArgsError::MissingValue { .. }),
                "{argv:?}: {err}"
            );
        }
    }

    #[test]
    fn invalid_param_rules_are_errors() {
        for rule in ["a", "a=1", ":=1", "a:=", ":a:=1", "a:={b: 1}", "a:=[1, x]"] {
            let err = RosArgs::parse(["prog", "--ros-args", "-p", rule]).unwrap_err();
            assert!(
                matches!(err, RosArgsError::InvalidParamRule { .. }),
                "{rule}: {err}"
            );
        }
    }

    #[test]
    fn unknown_ros_args_are_errors() {
        let err = RosArgs::parse([
            "prog",
            "--ros-args",
            "-r",
            "a:=b",
            "--log-level",
            "debug",
            "--",
            "--not-ros",
        ])
        .unwrap_err();
        assert_eq!(
            err,
            RosArgsError::UnknownArguments(vec![
                "-r".into(),
                "a:=b".into(),
                "--log-level".into(),
                "debug".into(),
            ])
        );
    }
}
