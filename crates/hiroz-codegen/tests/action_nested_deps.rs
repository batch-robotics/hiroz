//! Action Goal/Result/Feedback type hashing must include nested message
//! dependencies, the same way plain service hashing already does.
//!
//! `tf2_msgs/LookupTransform` exercises both affected paths with one real,
//! already-packaged fixture -- no synthetic types needed:
//! - Result (`geometry_msgs/TransformStamped transform`, `tf2_msgs/TF2Error
//!   error`): `TransformStamped` nests `std_msgs/Header` and
//!   `geometry_msgs/Transform`, and `Transform` itself nests
//!   `Vector3`/`Quaternion` -- multi-level nesting `calculate_get_result_hash`
//!   silently dropped from its dependency set.
//! - Goal (`builtin_interfaces/Duration timeout`, among primitives):
//!   `Duration` is a nested dependency `calculate_send_goal_hash` never
//!   inserted manually (unlike `builtin_interfaces/Time`, which every
//!   action hash function adds by hand for the goal_id UUID) -- it was only
//!   ever reachable via `collect_nested_deps`.
//!
//! Both were fixed by wiring `collect_nested_deps` into `resolver.rs`
//! (fj#158, GitHub#334).

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

#[test]
fn test_lookup_transform_hashes_include_nested_deps() {
    use hiroz_codegen::discovery::discover_actions;

    let assets = assets_dir();
    let packages = [
        "builtin_interfaces",
        "unique_identifier_msgs",
        "action_msgs",
        "service_msgs",
        "std_msgs",
        "geometry_msgs",
        "tf2_msgs",
    ];
    let mut all_messages = Vec::new();
    for pkg in &packages {
        let pkg_path = assets.join(pkg);
        let msgs = discover_messages(&pkg_path, pkg).unwrap_or_default();
        all_messages.extend(msgs);
    }

    let mut resolver = Resolver::new(false);
    resolver
        .resolve_messages(all_messages)
        .expect("resolve messages");

    let pkg_path = assets.join("tf2_msgs");
    let actions = discover_actions(&pkg_path, "tf2_msgs").expect("discover actions");
    let resolved = resolver.resolve_actions(actions).expect("resolve actions");
    let lookup_transform = resolved
        .iter()
        .find(|a| a.parsed.name == "LookupTransform")
        .expect("LookupTransform action");

    // Pinned after fixing calculate_get_result_hash to call
    // collect_nested_deps on the Result type, matching the plain-service
    // path (resolver.rs:181-182). Before that fix this hash is computed
    // over an incomplete dependency set (missing TransformStamped's own
    // nested Header/Transform/Vector3/Quaternion types) and differs from
    // this value -- this test must fail on unfixed code.
    assert_eq!(
        lookup_transform.get_result_hash.to_rihs_string(),
        "RIHS01_3cd1715751899e3167b5aec3e4ac194f7da9e8493a77285f9f6a4c914f5e8b24",
        "get_result hash mismatch -- nested dependency collection regressed"
    );

    // Same defect, the Goal side: calculate_send_goal_hash never inserted
    // builtin_interfaces/Duration -- the Goal's `timeout` field -- so it
    // was only ever reachable via collect_nested_deps too.
    assert_eq!(
        lookup_transform.send_goal_hash.to_rihs_string(),
        "RIHS01_4646ff5706c86b04d0a1098d329951a9731a7517e16702905de6073cc72c8530",
        "send_goal hash mismatch -- nested dependency collection regressed"
    );
}
