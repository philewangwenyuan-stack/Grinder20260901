#!/usr/bin/env python3
"""Restore pinned Livox sources locally without replacing existing device files."""
import argparse
import hashlib
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import urllib.request
import xml.etree.ElementTree as ET


SOURCES = (
    ('livox_ros_driver2', '13eb05e4e6dd7a765b934d0c5fd6236676a57b49',
     'https://codeload.github.com/Livox-SDK/livox_ros_driver2/tar.gz/13eb05e4e6dd7a765b934d0c5fd6236676a57b49',
     '4182a87ffa64f7c1e6bc9c1bafe0fff11023d97e3e9fbb271d09c7cbdb00c70e',
     ('CMakeLists.txt', 'cmake/version.cmake', 'src/driver_node.cpp',
      'msg/CustomMsg.msg', 'msg/CustomPoint.msg', 'package_ROS1.xml')),
    ('Livox_SDK2', 'v1.3.1',
     'https://codeload.github.com/Livox-SDK/Livox-SDK2/tar.gz/refs/tags/v1.3.1',
     '2e0f158cb1f473fe26bdacb9e292fbd31a2fba7f0177951818f95852a8920582',
     ('CMakeLists.txt', 'sdk_core/CMakeLists.txt', 'include/livox_lidar_api.h')),
)


def verified_archive(path, url, expected_sha256):
    path.parent.mkdir(parents=True, exist_ok=True)
    candidate = path
    if not path.is_file():
        candidate = path.with_suffix(path.suffix + '.partial')
        print('[INFO] Downloading ' + url, flush=True)
        with urllib.request.urlopen(url, timeout=60) as response, candidate.open('wb') as output:
            shutil.copyfileobj(response, output)
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    if digest != expected_sha256:
        raise ValueError('Livox archive SHA-256 mismatch: ' + str(candidate))
    if candidate != path:
        candidate.replace(path)
    return path


def restore_missing(archive_path, destination):
    """Validate the whole archive before writing; existing files are preserved."""
    destination = destination.resolve()
    with tarfile.open(archive_path, 'r:gz') as archive:
        entries = []
        roots = set()
        for member in archive.getmembers():
            name = PurePosixPath(member.name)
            if name.is_absolute() or '..' in name.parts or not (member.isdir() or member.isfile()):
                raise ValueError('Unsafe Livox archive member: ' + member.name)
            if not name.parts:
                continue
            roots.add(name.parts[0])
            if len(name.parts) > 1:
                target = destination.joinpath(*name.parts[1:])
                target.resolve().relative_to(destination)
                entries.append((member, target))
        if len(roots) != 1:
            raise ValueError('Livox archive must have exactly one source root')
        for member, target in entries:
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif not target.exists():
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, target.open('xb') as output:
                    shutil.copyfileobj(source, output)
                target.chmod(member.mode & 0o777)


def prepare(workspace):
    vendor = workspace / 'src/drivers/ws_livox/src'
    cache = workspace.parent / 'temp/grinder_livox_sources'
    for name, revision, url, digest, required in SOURCES:
        destination = vendor / name
        if not all((destination / item).is_file() for item in required):
            archive = verified_archive(cache / (name + '-' + revision + '.tar.gz'), url, digest)
            restore_missing(archive, destination)
            if not all((destination / item).is_file() for item in required):
                raise ValueError('Incomplete Livox source after restoration: ' + str(destination))
            print('[INFO] Restored {} {} (existing files preserved)'.format(name, revision))
        else:
            print('[INFO] Reusing existing Livox source: ' + str(destination))
    driver = vendor / 'livox_ros_driver2'
    manifest = driver / 'package.xml'
    if not manifest.exists():
        shutil.copyfile(driver / 'package_ROS1.xml', manifest)
    package = ET.parse(manifest).getroot()
    if package.findtext('name') != 'livox_ros_driver2' or package.findtext('buildtool_depend') != 'catkin':
        raise ValueError('Expected ROS1 package.xml; select package_ROS1.xml in ' + str(driver))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        prepare(args.workspace.resolve())
    except (OSError, ValueError, tarfile.TarError, ET.ParseError) as error:
        parser.exit(1, '[ERROR] {}\n'.format(error))


if __name__ == '__main__':
    main()
