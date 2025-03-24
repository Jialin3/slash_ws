#!/bin/bash

# 模型 URL 列表
model_urls=(
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/subt_tunnel_staging_area"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Jersey Barrier"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Base Station"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Fiducial"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Tunnel Tile 2"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Tunnel Tile 5"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Rescue Randy Sitting"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Black and Decker Cordless Drill"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Fire Extinguisher"
  "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Samsung J8 Black"
)

# 下载所有模型
for url in "${model_urls[@]}"; do
  ign fuel download -u "$url" 
done