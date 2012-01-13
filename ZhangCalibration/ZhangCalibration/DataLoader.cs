using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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
				for (int i = 0; i < 4; i++)
				{
					NotifyablePoint notifyablePoint = new NotifyablePoint();
					notifyablePoint.Point = new System.Windows.Point(numbers[ix++], numbers[ix++]);
					quad.Points.Add(notifyablePoint);
				}
				result.Add(quad);
			}

			return result;
		}

	}
}
