using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace IkRobotController
{
    class IKRCWrapper
    {
        private static bool isWrapped;

        public interface IKRCAPI
        {
            bool IsInitedModule { get; }
        }
    }
}
