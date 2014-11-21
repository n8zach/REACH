using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ArmController
{
    public struct ServoData
    {
        public int Id { get; private set; }
        public bool Moving { get; private set; }
        public int Position { get; set; }
        public int Torque { get; set; }
        public int Temp { get; set; }

        public ServoData(int id, bool moving, int position, int torque, int temp) : this()
        {
            this.Id = id;
            this.Moving = moving;
            this.Position = position;
            this.Torque = torque;
            this.Temp = temp;
        }
    }
}
