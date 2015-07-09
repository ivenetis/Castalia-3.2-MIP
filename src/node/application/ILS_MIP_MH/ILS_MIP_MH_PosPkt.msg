cplusplus {{
#include "ApplicationPacket_m.h"
}}

class ApplicationPacket;

namespace ILS_MIP_MH_NS;

struct PosPktData {
	int	nodeID;
	double	xCoord;
	double	yCoord;
}

packet PosPkt extends ApplicationPacket {
	PosPktData extraData;
}

