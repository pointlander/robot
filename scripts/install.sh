#!/bin/sh

cp robot.service /etc/systemd/system/
systemctl enable robot.service
