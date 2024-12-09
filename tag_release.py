import argparse
import subprocess
from pathlib import Path

from git import Repo
from loguru import logger


def main():
    logger.info("running pre-release preparations...")
    repo = Repo(Path('./'))
    if repo.is_dirty():
        logger.error(f"Found uncommited changes. Commit all changes before release!")
        return False

    parser = argparse.ArgumentParser(description="pre release scripts")
    parser.add_argument("--version", type=str, help="package version string")

    args = parser.parse_args()

    # check version string format
    version_str = args.version
    version_components = version_str.split(".")
    ver_YYMMDD = version_components[0]
    ver_fix = version_components[1]
    ver_rc = version_components[2]

    if not ver_YYMMDD.startswith('v'):
        logger.error(f"version string must start with vYYMMDD format, i.e. v240707.1.rc1")
        return False

    YYMMDD_str = ver_YYMMDD.strip('v')
    try:
        ymd_int = int(YYMMDD_str)
    except Exception as e:
        logger.warning(e)
        logger.error(f"version string must start with vYYMMDD format, i.e. v240707.1.rc1")
        return False

    if ymd_int < 240101 or ymd_int > 991231:
        logger.error(f"version string must start with vYYMMDD format, i.e. v240707.1.rc1")
        return False

    if len(YYMMDD_str) != 6:
        logger.error(f"version string must start with vYYMMDD format, i.e. v240707.1.rc1")
        return False
    
    try:
        int(ver_fix)
    except Exception as e:
        logger.warning(e)
        logger.error(f"the fix component in version string must be int, i.e. v240707.1.rc1")
        return False

    if not ver_rc.startswith("rc"):
        logger.error(f"rc1/2/3.. must be the last component, i.e. v240707.1.rc1")
        return False

    # find the version file and write version to it and commit it
    logger.info(f"update and commit version file by {version_str}")
    for path in Path('./src/').iterdir():
        if path.name.startswith('mx'):
            version_file = path / "__version__.py"
            with open(version_file, "w") as vf:
                vf.write(f"__version__ =\"{args.version}\"")
    repo.index.add([version_file])
    repo.index.commit(f"update version file by {version_str}")
    origin = repo.remotes.origin
    origin.push().raise_if_error()


    logger.info(f"tagging locally and remote...")
    try:
        subprocess.check_output(['git', 'tag', f'{version_str}',])
        subprocess.check_output(['git', 'push', '--tags',])
    except Exception as e:
        logger.warning(e)


    logger.info(f"now we are ready to build")

    return True

if __name__ == "__main__":
    if main():
        exit(1)
    else:
        exit(0)