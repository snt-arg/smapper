import os
from typing import List

from calib_toolbox.executor import JobPool
from calib_toolbox.logger import logger


class RosbagsConverter:
    def __init__(self, rosbags_dir: str, parrallel_jobs: int = 1):
        self.rosbags_dir = rosbags_dir
        self.ros1_bags_dir = os.path.join(rosbags_dir, "ros1")
        self.ros2_bags_dir = os.path.join(rosbags_dir, "ros2")

        self.jobs_pool = JobPool(parrallel_jobs)

    def convert(self) -> bool:
        if not self._validate_rosbags_dir():
            return False

        logger.info("Searching for ros2 bags to be converted...")

        for bag in os.listdir(self.ros2_bags_dir):
            src = os.path.join(self.ros2_bags_dir, bag)
            dest = os.path.join(self.ros1_bags_dir, f"{bag}.bag")
            if os.path.isfile(dest):
                logger.debug(f"{bag} has already been converted.")
                continue
            logger.info(f"Found ros2 bag {bag}")
            self.jobs_pool.add_job(self._build_cmd(src, dest))

        return self.jobs_pool.run_jobs(
            f"Converting {len(self.jobs_pool.jobs)} ros2 bags"
        )

    def _validate_rosbags_dir(self) -> bool:
        # Check if rosbags directory exists, and structure is correct
        # rosbags_dir/
        #   |- ros1/
        #   |- ros2/

        if not os.path.isdir(self.rosbags_dir):
            logger.error(f"Rosbags {self.rosbags_dir} directory does not exist")
            return False

        if not os.path.isdir(self.ros2_bags_dir):
            logger.error(
                f"""Ros2 bags {self.ros2_bags_dir} directory does not exist. 
                Make sure you place ros2 bags inside {self.ros2_bags_dir}"""
            )
            return False

        if not os.path.isdir(self.ros1_bags_dir):
            logger.info(
                f"Ros1 bags {self.ros1_bags_dir} directory does not yet exist. Creating it."
            )
            os.makedirs(self.ros1_bags_dir)

        return True

    def _build_cmd(self, src: str, dest: str) -> List[str]:
        return ["rosbags-convert", "--src", src, "--dst", dest]
