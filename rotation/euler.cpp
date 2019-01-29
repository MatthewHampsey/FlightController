#include "rotation.h"
#include <cmath>

namespace Penguin{
    Rotation ZYXEulerToRotation(float yaw, float pitch, float roll){
        //yaw mat:   [  cos  -sin  0    ]
	    //           [ sin   cos   0    ]
	    //           [ 0     0     1    ]
	    //pitch mat: [ cos   0     sin  ]
	    //           [ 0     1     0    ]
	    //           [ -sin  0     cos  ]
	    //roll mat:  [ 1     0     0    ]
	    //           [ 0     cos   -sin ]
	    //           [ 0     sin   cos  ]
	    //result: [cos(y)cos(p)  -sin(y)  cos(y)sin(p)][1 0   0   ]
	    //        [sin(y)cos(p)  cos(y)   sin(y)sin(p)][0 cos -sin]
	    //        [-sin(p)       0        cos(p)      ][0 sin cos ]
	    //
	    //        [cos(y)cos(p)   -sin(y)cos(r)+cos(y)sin(p)sin(r)    sin(y)sin(r)+cos(y)sin(p)cos(r)    ]
	    //        [sin(y)cos(p)   cos(y)cos(r)+sin(y)sin(p)sin(r)     -cos(y)sin(r) + sin(y)sin(p)cos(r) ]
	    //        [-sin(p)        cos(p)sin(r)                        cos(p)cos(r)                       ]
    
	    auto cosy = std::cos(yaw);
	    auto siny = std::sin(yaw);
	    auto cosp = std::cos(pitch);
	    auto sinp = std::sin(pitch);
	    auto cosr = std::cos(roll);
	    auto sinr = std::sin(roll);
        return fromMatrix({cosy*cosp, -siny*cosr+cosy*sinp*sinr, siny*sinr+cosy*sinp*cosr, 
		                   siny*cosp, cosy*cosr+siny*sinp*sinr,  -cosy*sinr+siny*sinp*cosr,
		                   -sinp,     cosp*sinr,                 cosp*cosr});
    }

}