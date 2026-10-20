import os
from pathlib import Path
import rclpy
from nav2_msgs.srv import LoadMap
rclpy.init();n=rclpy.create_node('reload_patrol_map');c=n.create_client(LoadMap,'/patrol_map_server/load_map')
assert c.wait_for_service(timeout_sec=5)
f=c.call_async(LoadMap.Request(map_url=str(Path(os.environ.get('YZBOT_WS',Path(__file__).resolve().parent))/'src/yzbot/bot_navigation/maps/competition_safe_map.yaml')))
rclpy.spin_until_future_complete(n,f,timeout_sec=10)
assert f.done();print('Map reload result:',f.result().result)
n.destroy_node();rclpy.shutdown()
