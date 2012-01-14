using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;
using System.Windows.Media;
using System.Windows;

namespace ZhangCalibration
{
	class DataLoader
	{

		public static List<Quad> ParseData(string data)
		{
			List<Quad> result = new List<Quad>();
			List<double> numbers = new List<double>();
			string[] split = data.Split(' ');

			for (int i = 0; i < split.Length; i++)
			{
				double d;
				if (double.TryParse(split[i].Replace('.', ','), out d))
				{
					numbers.Add(d);
				}
			}

			int ix = 0;
			while (ix + 8 <= numbers.Count)
			{
				Quad quad = new Quad();
				PointCollection points = new PointCollection();
				for (int i = 0; i < 4; i++)
				{
					Point point = new Point(numbers[ix++], numbers[ix++]);
					points.Add(point);
				}
				quad.Points = points;
				result.Add(quad);
			}

			return result;
		}

	}
}
