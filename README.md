![GLIM](docs/assets/logo2.png "GLIM Logo")

Thanks to Koide-san works!

Изменения в основном проекте GLIM

0) Добавлена библиотека PDAL для возможности сохранения в формате LAS

1) 

варианты сохранения фреймов с целью восстановленя картины исходных данных - все очень мутно - пользоваться нельзя
  //another_save_ply(path);
  //another_save_ply_thicker(path);
  //another_save_ply_extended(path);
  //another_save_las(path);
сохранения траектории
  //save_trajectory_ply(path); - не имеет особого смысла, если используется evo tools
  //save_trajectory_text(path); - фактически повторяет traj_lidar.txt
  
  
 сохраняем кейфреймы сразу - без использования dump & offline_viewer
  based_on_legacy_save_ply(path);
  
 2)
  Про повторном запуске с ключом командной строки reconstruct - на основе данных
  из /tmp/dump и читаемого points из баг файла ищем точки, близкие к keyframes (c учетом состоявшегося трансформа) -
  дополняем имеющиеся данные , вычисляем нормали и сохраняем в LAS
  
    GpsTime
    X
    Y
    Z
    Intensity
    NormalX
    NormalY
    NormalZ
 
 3)
 
 сохраняем кейфреймы сразу - без использования dump & offline_viewer - in LAS format с вычисленным приблизительно временем
  
  based_on_legacy_save_las(path);
 

