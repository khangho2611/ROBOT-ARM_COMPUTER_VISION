from pathlib import Path
import os
import subprocess

Import("env")


def run_camera_after_upload(target, source, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    runner = project_dir / "run_camera_base_d11.bat"

    if not runner.exists():
        print(f"[post-upload] Khong tim thay file: {runner}")
        return

    print("[post-upload] Upload xong. Dang mo camera_base_d11.py...")

    if os.name == "nt":
        subprocess.Popen(
            ["cmd", "/c", "start", "Camera Base D11", str(runner)],
            cwd=project_dir,
        )
        return

    subprocess.Popen(["py", "-3", "-u", "camera_base_d11.py"], cwd=project_dir)


env.AddPostAction("upload", run_camera_after_upload)
