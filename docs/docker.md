# Running Handy in Docker (ROS 2 Humble)

The development environment is a ROS 2 Humble desktop in a container, used through the browser
(noVNC). It works the same on Apple Silicon Macs (arm64) and Linux PCs (amd64).

| | |
|---|---|
| Base image | [`tiryoh/ros2-desktop-vnc:humble`](https://github.com/Tiryoh/docker-ros2-desktop-vnc): Ubuntu 22.04, ROS 2 Humble desktop, MATE, TigerVNC + noVNC |
| Added | MoveIt 2 + MoveIt Servo, ros2_control + controllers, xacro, Robotis DynamixelSDK, joint_state_publisher_gui, GitHub CLI, plus everything the packages' `package.xml` files declare (via rosdep) |
| Desktop | http://localhost:6080 (user `ubuntu`, password `ubuntu` unless `HANDY_PASSWORD` is set) |
| Workspace | the host folder holding `src/ivaHandy` is mounted at `~/handy_ws`; builds stay on the host |
| Home | `/home/ubuntu` is a named volume (`handy_home`), so Claude Code, `~/.claude`, the `gh` login, git config and `~/.bashrc` survive rebuilds |
| `/dev/shm` | 1 GB (Fast DDS shared memory; Docker's default of 64 MB is too small) |

Files: [`docker/Dockerfile`](../docker/Dockerfile), [`docker-compose.yml`](../docker-compose.yml),
[`docker-compose.hardware.yml`](../docker-compose.hardware.yml), [`.dockerignore`](../.dockerignore).

## Quick start

```bash
# on the host: the repo must sit at <workspace>/src/ivaHandy, with its submodule
mkdir -p ~/handy_ws/src && cd ~/handy_ws/src
git clone --recurse-submodules -b motor_implementation https://github.com/mistone-eng/ivaHandy.git
cd ivaHandy
docker compose up -d --build
```

Open http://localhost:6080, start a terminal on the desktop, then:

```bash
handy build        # builds ~/handy_ws (first time)
handy sim          # full stack on mock hardware, RViz on the desktop
handy demo         # (second terminal) MoveIt Servo demo; `handy wave` for the sine-wave demo
```

Stop with `docker compose down`; start again with `docker compose up -d`. After changing the
Dockerfile or any `package.xml` dependencies, rebuild with `docker compose up -d --build`.

## Real arm (Linux hosts only)

Docker Desktop on macOS and Windows cannot pass USB serial devices into containers, so the real arm
needs a Linux host (e.g. the lab PC) or a native Ubuntu 22.04 + Humble install.

```bash
# on the Linux host, once per boot: 1 ms USB latency, or 100 Hz control is impossible
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

docker compose -f docker-compose.yml -f docker-compose.hardware.yml up -d --build
```

`docker-compose.hardware.yml` passes `/dev/ttyUSB0` in and makes it writable for the desktop user
(Docker Compose 2.30 or newer; with older versions run `sudo chmod a+rw /dev/ttyUSB0` once in the
container). Then follow the real-arm steps in the README (`handy motors scan`, `handy calibrate zero`,
`handy real test`).

## Moving from an existing container

A container started by hand (`docker run …`) keeps the home folder inside the container, so it is
lost when the container is removed. To carry Claude Code, `~/.claude`, the `gh` login and git config
over to the new `handy_home` volume, copy them out first (replace `<old>` with the old container's
name from `docker ps -a`):

```bash
mkdir -p /tmp/handy_home
for d in .claude .claude.json .local .config/gh .gitconfig .bashrc; do
  docker cp "<old>:/home/ubuntu/$d" "/tmp/handy_home/$(dirname "$d")/" 2>/dev/null
done
docker compose up -d --build
docker cp /tmp/handy_home/. handy:/home/ubuntu/
docker compose exec -u root handy chown -R ubuntu:ubuntu /home/ubuntu
```

The workspace itself is on the host, so nothing needs copying for the code.

## Troubleshooting

* **Nodes hang right after "Could not enable FIFO RT scheduling"**: stale Fast DDS shared memory from
  killed processes. Run `handy kill`, then `fastdds shm clean`, then start again.
* **`handy sim` refuses to start**: another stack is still running; `handy kill`.
* **Port 6080 in use**: change the left side of `"6080:80"` in `docker-compose.yml`.
* **Container can't see `src/ivaHandy/third_party/ivaDynamixel`**: the submodule wasn't fetched;
  run `git submodule update --init` on the host.

The original ROS 1 (kinetic/noetic) Docker setup is in [`legacy/ros1/`](../legacy/ros1/README.md).
