You must set the `COMPOSE_PROJECT_NAME` environment variable to have the value `gisnav` at minimum for the shell that you are running `docker compose` commands from. It is important that all Docker image and container names have the string `gisnav` in them since for example the `expose-xhost` Makefile recipe depends on this string being present to expose the X server only to the appropriate containers.

::: code-group

  ```bash [Persistent <Badge type="tip" text="Recommended"/>]
  echo "export COMPOSE_PROJECT_NAME=gisnav" >> ~/.bashrc
  source ~/.bashrc
  ```
  ```bash [Current session]
  export COMPOSE_PROJECT_NAME=gisnav
  ```

  ```bash [Current shell only]
  COMPOSE_PROJECT_NAME=gisnav
  ```

:::
