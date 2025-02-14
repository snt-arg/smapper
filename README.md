# smapper

SMapper is a handheld device equipped with cameras and a 3D LiDAR which can be mounted on a robot.

## Documentation

Documentation can be built using docker or material mkdocs pip package.

### Docker

- For docker use `docker run --rm -it -p 8000:8000 -v ${PWD}:/docs squidfunk/mkdocs-material` on the root of the project.

### PIP

Use `pip install mkdocs-material` to install mkdocs and use `mkdocs server` to preview documentation.
