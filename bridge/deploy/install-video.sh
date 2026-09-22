#!/usr/bin/env bash
set -euo pipefail
# Run as the project owner; sudo only for packages and unit installation.
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
owner="$(id -un)"
if [[ "$owner" == root ]]; then echo 'Run as tieta, not root.'; exit 1; fi
sudo apt-get update
sudo apt-get install -y curl ca-certificates gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-rtsp
case "$(uname -m)" in x86_64) arch=amd64;; aarch64) arch=arm64;; *) echo 'Unsupported architecture'; exit 1;; esac
version=v1.21.0
asset="mediamtx_${version}_linux_${arch}.tar.gz"
mkdir -p "$project/.video/download" "$project/.video/bin"
cd "$project/.video/download"
curl --fail --location --retry 3 -o "$asset" "https://github.com/bluenviron/mediamtx/releases/download/$version/$asset"
curl --fail --location --retry 3 -o checksums.sha256 "https://github.com/bluenviron/mediamtx/releases/download/$version/checksums.sha256"
grep -F "$asset" checksums.sha256 > selected.sha256
sha256sum -c selected.sha256
tar -xzf "$asset" -C "$project/.video/bin" mediamtx
chmod +x "$project/.video/bin/mediamtx" "$project/deploy/publish-video.sh"
for element in souphttpsrc multipartdemux jpegdec x264enc h264parse rtspclientsink; do gst-inspect-1.0 "$element" >/dev/null; done
cat > "$project/.video/robot-video-server.service" <<EOF
[Unit]
Description=Robot WebRTC gateway
After=network-online.target
[Service]
User=$owner
WorkingDirectory=$project
ExecStart=$project/.video/bin/mediamtx $project/deploy/mediamtx.yml
Restart=on-failure
RestartSec=2
NoNewPrivileges=true
[Install]
WantedBy=multi-user.target
EOF
cat > "$project/.video/robot-video-publisher.service" <<EOF
[Unit]
Description=Robot H264 bounded-rate publisher
After=robot-video-server.service
Requires=robot-video-server.service
[Service]
User=$owner
WorkingDirectory=$project
ExecStart=/bin/bash $project/deploy/publish-video.sh
Restart=always
RestartSec=3
NoNewPrivileges=true
[Install]
WantedBy=multi-user.target
EOF
sudo install -m 644 "$project/.video/robot-video-server.service" "$project/.video/robot-video-publisher.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-video-server robot-video-publisher
echo 'Video services installed. Keep robot_bridge running. Check: journalctl -u robot-video-publisher -n 50'
