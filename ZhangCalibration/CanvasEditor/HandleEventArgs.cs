using System;

namespace CanvasEditor
{
	public class HandleEventArgs : EventArgs
	{

		public HandleEventArgs(IEditableHandle handle)
		{
			EditableHandle = handle;
		}

		public IEditableHandle EditableHandle { get; private set; }

	}
}
