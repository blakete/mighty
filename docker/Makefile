# mighty-hw: the MIGHTY hardware image. See README.hw.md.
#
# The image runs the mighty NODES in one idling container; mighty_hw.sh builds a
# HOST tmux session that docker-execs one node per pane, attached to the host
# zenoh router (drive.service on the RR fleet, :7447). Dependencies are cloned
# inside the build from ../mighty.repos; cloning the private mpc repo needs SSH:
# the targets forward your ssh-agent, or with no agent point SSH_KEY at a
# passphrase-free key:
#     SSH_KEY=$HOME/.ssh/id_ed25519 make hw-build
HW_COMPOSE = docker compose -f compose.hw.yaml
SSH_FWD    = --ssh default$(if $(SSH_KEY),=$(SSH_KEY))

.PHONY: hw-build hw-test hw-up hw-down hw-attach hw-status hw-pull
hw-build:  ; $(HW_COMPOSE) build $(SSH_FWD)
# Builds the mighty-test stage (BUILD_TESTING=ON + colcon test); fails on a test
# failure. Does not touch mighty-hw:local.
hw-test:   ; docker build $(SSH_FWD) -f Dockerfile.hw --target mighty-test ..
hw-up:     ; ./mighty_hw.sh start
hw-down:   ; ./mighty_hw.sh stop
hw-attach: ; tmux attach -t hw_mighty
hw-status: ; ./mighty_hw.sh status
hw-pull:   ; ./mighty_hw.sh pull $(TAG)
