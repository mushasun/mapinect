using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PcdLoader
{
    public class Color
    {
        public int R { get; set; }
        public int G { get; set; }
        public int B { get; set; }

        public Color()
        {
            Random rand = new Random();
            R = rand.Next(0, 255);
            G = rand.Next(0, 255);
            B = rand.Next(0, 255);
        }
    }
}
