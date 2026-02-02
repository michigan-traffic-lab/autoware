"""
Autoware Traffic Light Plugin (HTTP)

Syncs traffic light states from TeraSim (HTTP API) to Autoware.

Map: Ann Arbor NCRC Circular Route
- 31 SUMO TLS on AV route
- Regulatory elements: 674179-675445
"""

import requests
import rclpy
from rclpy.node import Node
from autoware_perception_msgs.msg import (
    TrafficSignal,
    TrafficSignalArray,
    TrafficSignalElement,
)


# Ann Arbor NCRC Circular Route mapping
# SUMO TLS ID -> List of (Autoware regulatory element ID, SUMO state string index)
#
# FIXED 2026-01-26: Corrected indices based on SUMO network connection analysis
# Key fixes:
#   - 62477148: index 4 (straight-through for 8974919181#0), was 3
#   - 62532012: index 1 (straight-through for 4411907471#0), was 4
#   - 767530322: index 5 (straight-through for 4411427181#0), was 9
#   - 62501598: index 0 (straight-through for 87339791#0), was 8
#   - 62527484: index 1 (straight-through for 87280471#0), was 7
#   - 62501419: index 1 (straight-through for 4414482250#0), was 8
#   - 62490467: index 1 (straight-through for 4414481701#0), was 4
#   - 62514378: index 1 (straight-through for 4126975921#0), was 4
#   - 62563769: index 1 (straight-through for 87344510#0), was 12
#   - 62486064: index 1 (straight-through for 4414482290#0), was 4
#   - 62601699: index 8 (straight-through for 2138799460#0), was 4
#   - 62494762: index 0 (straight-through for 4100274341#0), was 5
#   - 62500943: index 5 (straight-through for 4420466770#0), was 8
#   - 62537809: index 6 (straight-through for 4414481380), was 14
#   - cluster_62477163_62500824: index 7 (straight-through for 4117050240), was 11
SUMO_TO_AUTOWARE_TLS_MAPPING = {
    # 62477148: Controls intersection at 8974919181#0 (Plymouth Rd)
    # Straight-through linkIndex=4 from SUMO network analysis
    '62477148': [(674201, 4), (674283, 4), (674379, 4), (674477, 4), (674529, 4), (674591, 4), (674593, 4), (674695, 4), (674697, 4), (674699, 4), (674701, 4), (674703, 4), (674705, 4), (674841, 4), (674843, 4), (674845, 4), (674857, 4), (674859, 4), (674861, 4), (674863, 4), (674865, 4), (674867, 4), (675003, 4), (675005, 4), (675019, 4), (675099, 4), (675101, 4), (675201, 4), (675329, 4), (675333, 4), (675335, 4), (675337, 4), (675339, 4), (675341, 4), (675343, 4), (675381, 4), (675383, 4), (675385, 4), (675387, 4), (675389, 4)],
    # 62486064: Controls intersection at 4414482290#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62486064': [(674799, 1), (674801, 1), (674807, 1), (674809, 1), (675287, 1), (675355, 1)],
    '62487316': [(674325, 0), (674327, 0), (675103, 0), (675105, 0), (675143, 0)],
    # 62490467: Controls intersection at 4414481701#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62490467': [(674497, 1), (674753, 1), (674755, 1), (674757, 1), (674759, 1), (674761, 1), (674763, 1), (675173, 1)],
    # 62493581: Controls intersection at 4120499421#0
    # Straight-through linkIndex=14 from SUMO network analysis
    '62493581': [(674555, 14), (674557, 14), (674559, 14), (674561, 14), (674999, 14), (675001, 14), (675033, 14), (675035, 14), (675037, 14), (675153, 14)],
    # 62494762: Controls intersection at 4100274341#0
    # Straight-through linkIndex=0 from SUMO network analysis
    '62494762': [(674261, 0), (674263, 0), (674341, 0), (674513, 0), (674515, 0), (674517, 0), (674717, 0), (674719, 0), (675413, 0), (675435, 0), (675437, 0)],
    '62500567': [(674481, 0), (674483, 0), (674485, 0), (674623, 0), (674625, 0), (674665, 0), (674667, 0), (674669, 0)],
    # 62500943: Controls intersection at 4420466770#0
    # Straight-through linkIndex=5 from SUMO network analysis
    '62500943': [(674249, 5), (674377, 5), (674537, 5), (674539, 5), (674541, 5), (674543, 5), (674545, 5), (674917, 5), (674919, 5), (674921, 5), (674927, 5), (674929, 5), (674935, 5), (674937, 5), (674953, 5), (674955, 5), (675023, 5), (675069, 5), (675089, 5), (675091, 5), (675365, 5), (675367, 5)],
    # 62500958: Controls intersection at 44204667610#0
    # Straight-through linkIndex=5 from SUMO network analysis
    '62500958': [(674931, 5), (674933, 5), (675273, 5), (675275, 5), (675369, 5), (675371, 5), (675373, 5)],
    # 62501419: Controls intersection at 4414482250#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62501419': [(674195, 1), (674197, 1)],
    # 62501571: Controls intersection at 2227845261#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62501571': [(674345, 1), (674347, 1), (674349, 1), (674507, 1), (674509, 1), (674511, 1), (674825, 1), (674827, 1), (674829, 1), (674831, 1)],
    # 62501598: Controls intersection at 87339791#0 (Broadway & Plymouth)
    # Straight-through linkIndex=0 from SUMO network analysis
    '62501598': [(674343, 0), (674487, 0), (674641, 0), (674689, 0), (674691, 0), (674769, 0), (674771, 0), (674989, 0), (675165, 0), (675167, 0), (675293, 0), (675295, 0), (675297, 0)],
    # 62506219: Controls intersections at 4420319350#0 and 2135623741#0
    # Using linkIndex=0 for straight-through
    '62506219': [(674311, 0), (674313, 0), (674315, 0), (674923, 0), (674925, 0), (674977, 0), (674979, 0)],
    # 62514378: Controls intersection at 4126975921#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62514378': [(674393, 1), (674613, 1), (674615, 1), (674617, 1), (674733, 1), (674773, 1), (674775, 1), (674777, 1)],
    # 62517097: Controls intersection at 4421796661#0
    # Straight-through linkIndex=10 from SUMO network analysis
    '62517097': [(674991, 10), (674993, 10), (674995, 10), (674997, 10), (675047, 10), (675049, 10), (675051, 10), (675061, 10), (675063, 10), (675065, 10)],
    # 62527484: Controls intersection at 87280471#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62527484': [(674269, 1), (674301, 1), (674303, 1), (674305, 1), (674415, 1), (674581, 1), (674583, 1), (674731, 1), (674837, 1), (674875, 1), (674877, 1), (674881, 1), (674883, 1), (674885, 1), (674891, 1), (674893, 1), (675111, 1), (675113, 1), (675133, 1), (675163, 1), (675181, 1), (675183, 1), (675185, 1), (675223, 1), (675239, 1), (675241, 1), (675243, 1), (675289, 1), (675291, 1), (675301, 1)],
    # 62532012: Controls intersection at 4411907471#0 (Plymouth Rd)
    # Straight-through linkIndex=1 from SUMO network analysis
    '62532012': [(674309, 1), (674365, 1), (674367, 1), (674419, 1), (674479, 1), (674573, 1), (674595, 1), (674597, 1), (674599, 1), (674655, 1), (674661, 1), (674663, 1), (674685, 1), (674687, 1), (674707, 1), (674709, 1), (674711, 1), (674847, 1), (675021, 1), (675123, 1), (675127, 1), (675169, 1), (675175, 1), (675193, 1), (675195, 1), (675331, 1), (675345, 1), (675347, 1), (675395, 1)],
    # 62537809: Controls intersection at 4414481380
    # Straight-through linkIndex=6 from SUMO network analysis
    '62537809': [(674183, 6)],
    # 62537929: Controls intersections at 87349290#0 and 2577568310#0
    # Using linkIndex=14 for south direction (2577568310#0 straight-through)
    '62537929': [(674407, 14), (674409, 14), (674411, 14), (674905, 14), (674907, 14), (674909, 14), (675117, 14), (675119, 14), (675121, 14), (675315, 14), (675317, 14), (675319, 14)],
    # 62550570: Controls intersection at 87349231#0
    # Straight-through linkIndex=6 from SUMO network analysis
    '62550570': [(674215, 6), (674217, 6), (674219, 6), (674575, 6), (674577, 6), (674579, 6), (675227, 6), (675229, 6), (675231, 6), (675307, 6), (675309, 6), (675311, 6), (675313, 6)],
    # 62563769: Controls intersection at 87344510#0
    # Straight-through linkIndex=1 from SUMO network analysis
    '62563769': [(674619, 1), (674621, 1), (675251, 1), (675283, 1), (675299, 1)],
    # 62565682: Controls intersection at 4421796671#0
    # Straight-through linkIndex=16 from SUMO network analysis
    '62565682': [(675011, 16), (675013, 16), (675015, 16), (675017, 16), (675025, 16), (675027, 16), (675029, 16), (675031, 16), (675071, 16), (675073, 16), (675075, 16), (675077, 16), (675079, 16), (675081, 16)],
    '62578784': [(674321, 4), (674323, 4), (674473, 4), (674475, 4), (674519, 4)],
    # 62601699: Controls intersection at 2138799460#0
    # Straight-through linkIndex=8 from SUMO network analysis
    '62601699': [(674207, 8), (674209, 8)],
    # 62601733: Controls intersection at 3866940890#0
    # Straight-through linkIndex=10 from SUMO network analysis
    '62601733': [(674433, 10), (674435, 10), (674437, 10), (674961, 10), (674963, 10), (674965, 10), (674967, 10), (674969, 10), (674971, 10), (674973, 10)],
    # 62609606: Controls intersection at 4420319060#0
    # Straight-through linkIndex=14 from SUMO network analysis
    '62609606': [(674221, 14), (674223, 14), (674251, 14), (674253, 14), (674255, 14), (674899, 14), (674901, 14), (674903, 14)],
    # 767530322: Controls intersection at 4411427181#0 (Plymouth Rd)
    # Straight-through linkIndex=5 from SUMO network analysis
    '767530322': [(674351, 5), (674353, 5), (674355, 5), (674361, 5), (674363, 5), (674675, 5), (674677, 5), (674679, 5), (674693, 5)],
    # GS_11332562158: Controls intersection at 117246727410#0
    # Straight-through linkIndex=1 from SUMO network analysis
    'GS_11332562158': [(674211, 1), (674213, 1), (674461, 1), (674463, 1), (675189, 1), (675191, 1), (675213, 1), (675215, 1)],
    # cluster_62477163_62500824: Controls intersection at 4117050240
    # Straight-through linkIndex=7 from SUMO network analysis
    'cluster_62477163_62500824': [(674243, 7), (674245, 7), (674247, 7), (674369, 7), (674371, 7), (674373, 7), (674521, 7), (674523, 7), (674525, 7), (674527, 7), (674585, 7), (674587, 7), (674589, 7)],
    # cluster_62484035_62484037: Controls intersection at 2232850940
    # Straight-through linkIndex=5 from SUMO network analysis
    'cluster_62484035_62484037': [(674357, 5), (674359, 5), (674385, 5), (674387, 5), (674389, 5), (674391, 5), (675129, 5), (675131, 5), (675257, 5), (675259, 5), (675261, 5), (675263, 5)],
    # cluster_62500913_62548685: Controls intersection at 229035100
    # Straight-through linkIndex=5 from SUMO network analysis
    'cluster_62500913_62548685': [(674395, 5), (674397, 5), (674399, 5), (674401, 5), (674531, 5), (674533, 5), (674535, 5), (674647, 5), (674649, 5), (674651, 5), (674653, 5), (675217, 5), (675219, 5), (675221, 5)],
    # =========================================================================
    # MISSING TLS - Added 2026-01-26 (need regulatory element IDs if issues occur)
    # =========================================================================
    # 29311341: Controls intersection at 46185540 (U-turn)
    # Straight-through linkIndex=0 from SUMO network analysis
    '29311341': [],  # TODO: Add regulatory element IDs if needed
    # 62497500: Controls intersection at 3865684260#0
    # Straight-through linkIndex=16 from SUMO network analysis
    '62497500': [],  # TODO: Add regulatory element IDs if needed
    # 62578789: Controls intersection at 41162193930#0
    # Straight-through linkIndex=10 from SUMO network analysis
    '62578789': [],  # TODO: Add regulatory element IDs if needed
    # 62578796: Controls intersection at 41244134410#0
    # Straight-through linkIndex=9 from SUMO network analysis
    '62578796': [],  # TODO: Add regulatory element IDs if needed
}


class AutowareTLSPlugin(Node):

    def __init__(self, **kwargs):
        super().__init__("autoware_tls_plugin", **kwargs)

        # Declare parameters
        self.declare_parameter("http_host", "localhost")
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("simulation_id", "")

        self.http_host = self.get_parameter("http_host").value
        self.http_port = self.get_parameter("http_port").value
        self.simulation_id = self.get_parameter("simulation_id").value

        self.base_url = f"http://{self.http_host}:{self.http_port}"
        self.session = requests.Session()

        # Publisher
        self.pub_traffic_signals = self.create_publisher(
            TrafficSignalArray,
            "/perception/traffic_light_recognition/traffic_signals",
            10,
        )

        # Timer - 20Hz
        self.timer = self.create_timer(0.05, self.on_timer)

        # State cache
        self.last_state = None

        self.get_logger().info(f"TLS plugin started: {self.base_url}, sim_id={self.simulation_id}")

    def _get_state(self):
        """Get simulation state from TeraSim HTTP API."""
        if not self.simulation_id:
            return None
        try:
            resp = self.session.get(
                f"{self.base_url}/simulation/{self.simulation_id}/state",
                timeout=0.5
            )
            self.last_state = resp.json()
            return self.last_state
        except:
            return self.last_state

    def on_timer(self):
        """Publish traffic light states to Autoware."""
        state = self._get_state()
        if not state or "traffic_light_details" not in state:
            return

        signals = {}

        # Process each SUMO traffic light
        for tls_id, tls_info in state["traffic_light_details"].items():
            state_str = tls_info.get("tls", "") if isinstance(tls_info, dict) else tls_info

            # Map to Autoware regulatory elements
            for reg_id, lane_idx in SUMO_TO_AUTOWARE_TLS_MAPPING.get(tls_id, []):
                if reg_id not in signals:
                    # Get light state from SUMO state string
                    light = state_str[lane_idx] if lane_idx < len(state_str) else 'r'

                    elem = TrafficSignalElement()
                    elem.shape = TrafficSignalElement.CIRCLE
                    elem.status = TrafficSignalElement.SOLID_ON
                    elem.confidence = 1.0

                    # Map SUMO light state to Autoware color
                    color_map = {
                        'r': TrafficSignalElement.RED,
                        'y': TrafficSignalElement.AMBER,
                        'g': TrafficSignalElement.GREEN,
                        'G': TrafficSignalElement.GREEN,
                    }
                    elem.color = color_map.get(light, TrafficSignalElement.RED)

                    sig = TrafficSignal()
                    sig.traffic_signal_id = reg_id
                    sig.elements = [elem]
                    signals[reg_id] = sig

        # Publish
        msg = TrafficSignalArray()
        msg.stamp = self.get_clock().now().to_msg()
        msg.signals = list(signals.values())
        self.pub_traffic_signals.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutowareTLSPlugin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
