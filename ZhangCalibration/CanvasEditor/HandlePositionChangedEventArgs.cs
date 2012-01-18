using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;

namespace CanvasEditor
{
	public class HandlePositionChangedEventArgs : HandleEventArgs
	{
		public HandlePositionChangedEventArgs(IEditableHandle handle, Point position) : base(handle)
		{
			Position = position;
		}

		public Point Position { get; private set; }

	}
}
