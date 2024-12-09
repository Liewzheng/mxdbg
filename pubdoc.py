import os
import os.path
from pathlib import Path, PureWindowsPath

import paramiko
from loguru import logger


class PydocsFTP(object):
    def __init__(self) -> None:
        self.host = '192.168.1.103'
        self.port = 60722
        self.username = "sftp-pydocs"
        self.password = "Mixo-pydocs"
        pass

    def connect(self) -> bool:
        """链接外部ftp

        Returns:
            bool: 成功返回True
        """
        try:
            self.transport = paramiko.Transport((self.host, self.port))
            self.transport.connect(username=self.username, password=self.password)
            self.sftp = paramiko.SFTPClient.from_transport(self.transport)
            logger.info("SFTP server connected")
            return True
        except Exception as e:
            logger.error(f"SFTP server connection failed: {str(e)}")
            return False

    def upload(self, local_file_path: str, bucket_name: str, targetDir: str, filename: str) -> bool:
        """上传文件到ftp服务器

        Args:
            local_file_path (str): 本地需要上传的文件完整路径（包括文件名）
            bucket_name (str): 目标bucket name
            targetDir (str): 目标bucket下的需要保存的目录
            filename (str): 需要保存的文件名

        Returns:
            bool: 成功返回True
        """
        remote_file_path = '/Alfred/' + bucket_name + '/' + targetDir + '/' + filename
        remote_file_dir = '/Alfred/' + bucket_name + '/' + targetDir

        logger.info(f"trying to upload {local_file_path} to bucket {bucket_name} with path: {remote_file_path}")
        if not self.sftp:
            logger.warning("请先连接到SFTP服务器")
            return False

        # 检查本地路径,不存在跳过
        if not os.path.exists(local_file_path):
            logger.warning(f"本地文件不存在:{local_file_path}")
            return False

        try:
            # 检查远程文件路径,存在跳过
            logger.info(f"checking object path existance {remote_file_path}")
            self.sftp.stat(remote_file_path)
            logger.error("remote object exist, operation abort.")
            return False
        except Exception as e:
            logger.info(e)
            logger.info("remote object not exist, ready to transfer")

        # make all the dirs if they don't exist
        sub_dirs = remote_file_dir.strip('/').split('/')
        logger.debug(f"check all sub dir levels: {sub_dirs}")
        for i in range(0, len(sub_dirs), 1):
            dir2check = '/' + '/'.join(sub_dirs[0:i+1])
            logger.debug(f"checking {dir2check}")
            try:
                self.sftp.chdir(dir2check)
            except Exception as e:
                logger.info(e)
                logger.info("remote dir don't exist, creating it...")
                self.sftp.mkdir(dir2check)
                continue

        try:
            logger.info(f"try to upload from {local_file_path} to {remote_file_path}")
            if os.path.isfile(local_file_path):
                # upload
                self.sftp.put(local_file_path, remote_file_path)  # 上传文件
                logger.info("upload succeeded")
                return True
        except Exception as e:
            logger.error(f"upload error:{e}")
            return False
        
        return True

def main():
    logger.info("publishing doc site to pydocs.it.mixo.local")
    site_dir = Path("./site")
    src_dir = Path("./src")
    remote = PydocsFTP()
    remote.connect()
    package_name = list(src_dir.glob("mx*"))[0].name.lower()
    with open(list(src_dir.glob("mx*"))[0]/"__version__.py", 'r') as vf:
        content = vf.read()
        ver = content.split("=")[1].strip("\"")
    for afile in site_dir.rglob("*"):
        targetDir = str(PureWindowsPath(afile.parent).as_posix()).replace("site/", '').replace('site','')
        bucket_name = "/".join(["pydocs", package_name, ver])
        remote.upload(afile.absolute(), bucket_name=bucket_name, targetDir=targetDir, filename=afile.name)
    return True

if __name__ == "__main__":
    if main():
        exit(1)
    else:
        exit(0)
