import subprocess
import time
from typing import List, Optional
from dataclasses import dataclass
from enum import Enum
import signal
from rich.progress import (
    Progress,
    TimeElapsedColumn,
    BarColumn,
    TextColumn,
    TaskProgressColumn,
    SpinnerColumn,
)
from smapper_toolbox.logger import logger


class JobStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class JobResult:
    """Stores the result of a job execution"""

    returncode: int
    stdout: Optional[str]
    stderr: Optional[str]
    status: JobStatus


class Job:
    """Represents a single job in the execution pool"""

    def __init__(self, cmd: List[str], job_id: int):
        self.cmd = cmd
        self.id = job_id
        self.process: Optional[subprocess.Popen] = None
        self.status = JobStatus.PENDING
        self.result: Optional[JobResult] = None

    def start(self) -> None:
        """Start the job process"""
        try:
            self.process = subprocess.Popen(
                self.cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
            )
            self.status = JobStatus.RUNNING
        except Exception as e:
            logger.error(f"Failed to start job {self.id}: {str(e)}")
            self.status = JobStatus.FAILED
            self.result = JobResult(1, None, str(e), JobStatus.FAILED)

    def check_status(self) -> JobStatus:
        """Check the current status of the job"""
        if not self.process:
            return self.status

        returncode = self.process.poll()
        if returncode is not None:
            stdout, stderr = self.process.communicate()
            self.status = JobStatus.COMPLETED if returncode == 0 else JobStatus.FAILED
            self.result = JobResult(returncode, stdout, stderr, self.status)

            if self.status == JobStatus.FAILED:
                logger.error(f"Job {self.id} failed with return code {returncode}")
                if stderr:
                    logger.error(f"Error output: {stderr}")

        return self.status


class JobPool:
    """Manages a pool of jobs with parallel execution"""

    def __init__(self, max_parallel: int):
        self.max_parallel = max_parallel
        self.jobs: List[Job] = []
        self.active_jobs: List[Job] = []
        self.completed_jobs: List[Job] = []

    def add_job(self, cmd: List[str]) -> None:
        """Add a new job to the pool"""
        job = Job(cmd, len(self.jobs))
        self.jobs.append(job)

    def run_jobs(self, description: str) -> bool:
        """Run all jobs in the pool with progress tracking"""
        with Progress(
            SpinnerColumn(),
            TextColumn("[bold blue]{task.description}"),
            BarColumn(),
            TaskProgressColumn(),
            TimeElapsedColumn(),
            transient=True,
        ) as progress:
            task = progress.add_task(description, total=len(self.jobs))

            try:
                while self.jobs or self.active_jobs:
                    # Start new jobs if possible
                    while self.jobs and len(self.active_jobs) < self.max_parallel:
                        job = self.jobs.pop(0)
                        job.start()
                        if job.status != JobStatus.FAILED:
                            self.active_jobs.append(job)
                        else:
                            progress.update(task, advance=1)

                    # Check active jobs
                    for job in self.active_jobs[:]:
                        status = job.check_status()
                        if status in [JobStatus.COMPLETED, JobStatus.FAILED]:
                            self.active_jobs.remove(job)
                            self.completed_jobs.append(job)
                            progress.update(task, advance=1)

                    time.sleep(0.1)

                # Check if any jobs failed
                failed_jobs = [
                    job for job in self.completed_jobs if job.status == JobStatus.FAILED
                ]
                if failed_jobs:
                    logger.error(f"{len(failed_jobs)} jobs failed")
                    return False

                return True

            except KeyboardInterrupt:
                logger.warning("Received interrupt, cleaning up jobs...")
                self._cleanup_jobs()
                return False

    def _cleanup_jobs(self) -> None:
        """Clean up all running jobs"""
        for job in self.active_jobs:
            if job.process:
                try:
                    job.process.terminate()
                    job.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    job.process.kill()


def execute_pool(
    cmds: List[List[str]], description: str, max_parallel_jobs: int = 1
) -> bool:
    """
    Execute a pool of commands in parallel with improved error handling and job management.

    Args:
        cmds: List of commands to execute
        description: Description for the progress bar
        max_parallel_jobs: Maximum number of parallel jobs

    Returns:
        bool: True if all jobs completed successfully, False otherwise
    """
    pool = JobPool(max_parallel_jobs)
    for cmd in cmds:
        pool.add_job(cmd)
    return pool.run_jobs(description)
