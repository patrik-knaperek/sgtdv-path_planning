# Bakalarska praca
## Planovanie trajektorie pre autonomny monopost Formula Student

Pre zvolenie rezimu pathplanning je nutne zmenit parameter _FULL_MAP_   v subore **PathPlanning.h**


Pre standalone spustenie:
```sh
roslaunch path_planning_bp path_planning.launch
```

Pre spustenie s rosbagom (.bag file sa musi nachadzat vo folderi /bags):
```sh
roslaunch path_planning_bp path_planning_rosbag.launch bag_name:=YOUR_BAG_FILE
```



