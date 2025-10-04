Import("env")

import os
import shutil
import subprocess
import sys


def log(message: str) -> None:
    print("[override_teensy_cores] " + message)


def run(cmd, cwd=None) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def get_option(name: str, default: str | None = None) -> str | None:
    try:
        return env.GetProjectOption(name)
    except Exception:
        return default


def copy_merge_tree(src_root: str, dst_root: str) -> None:
    for root, dirs, files in os.walk(src_root):
        # Skip VCS directories
        if ".git" in dirs:
            dirs.remove(".git")
        rel = os.path.relpath(root, src_root)
        target_dir = dst_root if rel == "." else os.path.join(dst_root, rel)
        os.makedirs(target_dir, exist_ok=True)
        for f in files:
            src_path = os.path.join(root, f)
            dst_path = os.path.join(target_dir, f)
            shutil.copy2(src_path, dst_path)


def ensure_override() -> None:
    url = get_option("custom_teensy_cores_url")
    ref = get_option("custom_teensy_cores_ref", "main")
    if not url:
        log("custom_teensy_cores_url not set; skipping override.")
        return

    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoteensy")
    if not framework_dir or not os.path.isdir(framework_dir):
        log("framework-arduinoteensy not installed yet; skipping override this pass.")
        return

    cores_target = os.path.join(framework_dir, "cores")
    if not os.path.isdir(cores_target):
        log(f"cores directory not found in {framework_dir}; skipping override.")
        return

    build_dir = env.subst("$BUILD_DIR")
    override_dir = os.path.join(build_dir, "cores_override_repo")
    marker_path = os.path.join(framework_dir, ".override_teensy_cores.txt")

    try:
        if not os.path.exists(override_dir):
            os.makedirs(override_dir, exist_ok=True)
            log(f"Cloning {url} ({ref}) …")
            run(["git", "clone", "--depth", "1", "--branch", ref, url, override_dir])
        else:
            log(f"Updating {override_dir} to {ref} …")
            run(["git", "fetch", "--depth", "1", "origin", ref], cwd=override_dir)
            run(["git", "checkout", "-B", "override", "FETCH_HEAD"], cwd=override_dir)

        # Resolve the exact commit SHA
        rev = run(["git", "rev-parse", "HEAD"], cwd=override_dir).stdout.strip()
        log(f"Using cores commit {rev}")

        # Overlay repo root into installed cores/ directory
        log(f"Overlaying {override_dir} → {cores_target}")
        copy_merge_tree(override_dir, cores_target)

        with open(marker_path, "w", encoding="utf-8") as f:
            f.write(f"source: {url}\nref: {ref}\ncommit: {rev}\n")

        log("Override complete.")
    except subprocess.CalledProcessError as e:
        log("Command failed:\n" + e.stdout)
        # Do not fail the build due to override issues
    except Exception as e:
        log(f"Unexpected error: {e}")


# Run during the pre build stage (script is invoked with pre: prefix)
ensure_override()


