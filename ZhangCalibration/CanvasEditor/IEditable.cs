using System.Windows;
using PropertyChangedLibrary;

namespace CanvasEditor
{
	public interface IEditable
	{
		UIElement UIElement { get; }
		bool IsEditable { get; }
		bool IsEnabled { get; }

		PointNotifyPropertyChanged Position { get; }
		SizeNotifyPropertyChanged Size { get; }

		event OnEditableSizeChanged SizeChanged;
	}

	public delegate void OnEditableSizeChanged(object sender, EditableSizeEventArgs e);
}
