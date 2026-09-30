mod common;

use clap::Parser;
use hiroz::{
    Builder, Result,
    parameter::{ParameterDescriptor, ParameterType, ParameterValue},
    ros_args::RosArgs,
};

#[derive(Parser)]
#[command(about = "ROS 2 command-line parameter overrides demo")]
struct Args {
    /// Zenoh router endpoint (e.g., tcp/localhost:7447)
    #[arg(short, long)]
    endpoint: Option<String>,
}

// Try:
//   cargo run --example z_parameter_ros_args -- \
//     --ros-args -p max_speed:=2.5 -p robot_name:=rover --
fn main() -> Result<()> {
    common::init();

    // Split the ROS arguments off the process argv, as rclcpp does.
    let ros_args = RosArgs::from_env()?;
    // Everything outside `--ros-args ... --`, program name first.
    let args = Args::parse_from(ros_args.remaining_args());
    let ctx = common::create_context(args.endpoint)?;

    println!("\n=== ROS Arguments Demo ===\n");
    println!("params files: {:?}", ros_args.params_files());

    let node = ctx
        .create_node("ros_args_demo")
        .with_ros_args(&ros_args)
        .build()?;

    let desc = ParameterDescriptor::new("max_speed", ParameterType::Double);
    let max_speed = node
        .declare_parameter("max_speed", ParameterValue::Double(1.0), desc)
        .expect("declare max_speed");
    println!("max_speed = {:?} (default 1.0)", max_speed);

    // Command-line overrides apply to read-only parameters too; only later
    // runtime changes are rejected.
    let mut desc = ParameterDescriptor::new("robot_name", ParameterType::String);
    desc.read_only = true;
    let robot_name = node
        .declare_parameter("robot_name", ParameterValue::String("unnamed".into()), desc)
        .expect("declare robot_name");
    println!(
        "robot_name = {:?} (default \"unnamed\", read-only)",
        robot_name
    );

    Ok(())
}
