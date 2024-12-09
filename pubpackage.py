import os
import os.path
from pathlib import Path, PureWindowsPath

import paramiko
from loguru import logger


class PypiServersFTP(object):
    def __init__(self) -> None:
        self.host = '192.168.1.103'
        self.port = 60722
        self.username = "sftpuser"
        self.password = "Mixo-sftp"
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
        remote_file_path = targetDir + '/' + bucket_name + '/' + filename

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

def main() -> bool:
    logger.info("publishing the package to mxpypi.it.mixo.local")
    pkg_dir = Path("./dist")
    src_dir = Path("./src")
    remote = PypiServersFTP()
    remote.connect()
    with open(list(src_dir.glob("mx*"))[0]/"__version__.py", 'r') as vf:
        content = vf.read()
        version_split = content.split("=")[1].strip("\"").split('.')
        ver = version_split[0][1:] + '.' + version_split[1] + version_split[2]
    
    for afile in pkg_dir.rglob("*"):
        if ver in afile.name and afile.name.endswith('.whl'):
            targetDir = str(PureWindowsPath(afile.parent).as_posix()).replace("dist/", '').replace('dist','')
            remote.upload(str(afile.absolute()), bucket_name='packages', targetDir=targetDir, filename=afile.name)
    return True

if __name__ == "__main__":
    if main():
        exit(1)
    else:
        exit(0)
