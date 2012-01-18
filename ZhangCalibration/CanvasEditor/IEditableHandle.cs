using System.Windows;

namespace CanvasEditor
{
	public interface IEditableHandle
	{
		UIElement UIElement { get; }
		IEditable EditableElement { get; }
		BoundaryPosition BoundaryPosition { get; }
		bool IsVisible { get; set; }

		event OnHandlePositionChanged PositionChanged;
	}

	public enum BoundaryPosition
	{
		TopLeft = 0,
		TopCenter,
		TopRight,
		MidRight,
		BottomRight,
		BottomCenter,
		BottomLeft,
		MidLeft,
		BoundaryPositionCount
	}

	public delegate void OnHandlePositionChanged(object sender, HandlePositionChangedEventArgs e);

}
