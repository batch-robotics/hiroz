use std::path::{Path, PathBuf};

use hiroz_codegen::{
    discovery::discover_messages,
    parser::{action::parse_action, msg::parse_msg_string},
    resolver::Resolver,
    types::ResolvedAction,
};

fn assets_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/jazzy")
}

fn resolve_action(nested_definition: &str) -> ResolvedAction {
    let assets = assets_dir();
    let mut messages = Vec::new();
    for package in [
        "builtin_interfaces",
        "unique_identifier_msgs",
        "action_msgs",
        "service_msgs",
    ] {
        messages.extend(
            discover_messages(&assets.join(package), package)
                .unwrap_or_else(|error| panic!("discover {package}: {error}")),
        );
    }

    messages.push(
        parse_msg_string(
            nested_definition,
            "nested_action_interfaces",
            Path::new("Nested.msg"),
        )
        .expect("parse nested message"),
    );

    let mut resolver = Resolver::new(false);
    resolver
        .resolve_messages(messages)
        .expect("resolve messages");

    let action = parse_action(
        "Nested goal\n---\nNested result\n---\nNested feedback\n",
        "ExerciseNestedTypes",
        "nested_action_interfaces",
        Path::new("ExerciseNestedTypes.action"),
    )
    .expect("parse action");

    resolver.resolve_action(action).expect("resolve action")
}

#[test]
fn nested_types_contribute_to_all_action_protocol_hashes() {
    let original = resolve_action("int32 value\n");
    let changed = resolve_action("int32 value\nstring note\n");

    assert_ne!(original.send_goal_hash, changed.send_goal_hash);
    assert_ne!(original.get_result_hash, changed.get_result_hash);
    assert_ne!(
        original.feedback_message_hash,
        changed.feedback_message_hash
    );
}
