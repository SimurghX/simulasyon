ros bridge komutu(drone kamerası için) :ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image[gz.msgs.Image
ros bridge komutu(rover kamerası için) :ros2 run ros_gz_bridge parameter_bridge /model/r1/camera@sensor_msgs/msg/Image@gz.msgs.Image
ros bridge komutu(rover hareketi için) :ros2 run ros_gz_bridge parameter_bridge /model/r1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist



hi.py python dosyasını oluşturduğunuz ros2 ws in içinde çalıştırmanız tavsiye edilir.
