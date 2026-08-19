"""
Tests for uvx-based installation method.

This tests the primary documented installation method using uvx.
All tests install from git to validate before publishing.
"""

from pathlib import Path

import pytest

from .conftest import build_docker_image, cleanup_docker_image


@pytest.mark.installation
@pytest.mark.slow
def test_uvx_install_from_git(repo_root: Path, docker_dir: Path, git_branch: str, repo_url: str):
    """
    Test uvx ros-mcp installation from git.

    This verifies that uvx can install and run ros-mcp from a git branch.
    Uses: uvx --from git+REPO_URL@BRANCH ros-mcp --help
    """
    dockerfile = docker_dir / "Dockerfile.uvx"
    tag = "ros-mcp-test:uvx-git"

    try:
        result = build_docker_image(
            dockerfile_path=dockerfile,
            context_path=repo_root,
            tag=tag,
            build_args={
                "REPO_URL": repo_url,
                "BRANCH": git_branch,
            },
            timeout=300,  # 5 minutes for uv and package downloads
        )

        assert result.returncode == 0, (
            f"uvx installation from git failed (branch: {git_branch}):\n"
            f"STDOUT:\n{result.stdout}\n"
            f"STDERR:\n{result.stderr}"
        )

    finally:
        cleanup_docker_image(tag)


@pytest.mark.installation
@pytest.mark.slow
def test_uvx_install_from_local(repo_root: Path, docker_dir: Path):
    """
    Test uvx ros-mcp installation from local repository.

    This verifies that uvx can install and run ros-mcp from a local directory.
    Uses: uvx . ros-mcp --help
    """
    dockerfile = docker_dir / "Dockerfile.uvx-local"
    tag = "ros-mcp-test:uvx-local"

    try:
        result = build_docker_image(
            dockerfile_path=dockerfile,
            context_path=repo_root,
            tag=tag,
            timeout=300,  # 5 minutes for uv and package downloads
        )

        assert result.returncode == 0, (
            f"uvx installation from local failed:\n"
            f"STDOUT:\n{result.stdout}\n"
            f"STDERR:\n{result.stderr}"
        )

    finally:
        cleanup_docker_image(tag)
