using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PcdLoader
{
    public class PcdFile
    {
        public static Random rand = new Random(DateTime.Now.Second);

        public string fullName { get; set; }
        public string name { get; set; }
        public int R { get; set; }
        public int G { get; set; }
        public int B { get; set; }
        public int size { get; set; }
        public float opacity { get; set; }
        public DateTime date { get; set; }

        public PcdFile(string fullName, string name, DateTime date)
        {
            this.fullName = fullName;
            this.name = name;
            opacity = 1;
            size = 1;
            this.date = date;
            
            R = rand.Next(0, 255);
            G = rand.Next(0, 255);
            B = rand.Next(0, 255);

        }
    }
}
