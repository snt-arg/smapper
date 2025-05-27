import subprocess
import time
from typing import List

import typer
from rich.progress import (
    Progress,
    TimeElapsedColumn,
    BarColumn,
    TextColumn,
    TaskProgressColumn,
)

app = typer.Typer()

Command = List[str]
Commands = List[Command]


def execute_pool(cmds: Commands, description: str, max_parallel_jobs: int = 1):
    """
    Executes a list of shell commands in parallel with a maximum number of concurrent jobs,
    displaying progress using a Rich progress bar.

    Args:
        cmds (Commands): A list of commands, where each command is a list of strings
                         representing the command and its arguments (e.g., ["ls", "-l"]).
        description (str): A descriptive label shown with the progress bar.
        max_parallel_jobs (int): The maximum number of commands to run concurrently.

    Displays:
        A Rich progress bar showing the number of completed jobs out of the total,
        along with elapsed time and a custom description panel above the bar.
    """
    jobs: List[subprocess.Popen] = []
    completed = 0
    total = len(cmds)

    with Progress(
        TextColumn("[bold blue]{task.description}"),
        BarColumn(),
        TaskProgressColumn(),
        TimeElapsedColumn(),
        transient=True,
    ) as progress:
        task = progress.add_task(description, total=total)

        for cmd in cmds:
            jobs.append(subprocess.Popen(cmd, text=True))

            # Wait until the number of jobs is below the maximum
            while len(jobs) >= max_parallel_jobs:
                for i in range(len(jobs) - 1, -1, -1):
                    job = jobs[i]
                    if job.poll() is not None:
                        jobs.pop(i)
                        completed += 1
                        progress.update(task, advance=1)
                time.sleep(0.1)

        # Wait for remaining jobs
        while jobs:
            for i in range(len(jobs) - 1, -1, -1):
                job = jobs[i]
                if job.poll() is not None:
                    jobs.pop(i)
                    completed += 1
                    progress.update(task, advance=1)
            time.sleep(0.1)
