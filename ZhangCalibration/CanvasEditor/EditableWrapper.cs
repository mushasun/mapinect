using System;
using System.Windows;
using System.Reflection;

namespace CanvasEditor
{
	internal class EditableWrapper
	{

		static internal IEditable CreateWrapperForUIElement(CanvasEditor editor, UIElement element)
		{
			IEditable result = null;

			Assembly assembly = Assembly.GetAssembly(typeof(EditableWrapper));
			string name = element.GetType().Name;
			Type type = assembly.GetType(typeof(EditableWrapper).Namespace + ".Editable" + name);
			if (type != null)
			{
				result = (IEditable)Activator.CreateInstance(type, new object[] { editor, element });
			}

			return result;
		}

	}
}
