using System.Windows;
using System.Windows.Controls;
using PropertyChangedLibrary;
using System.Windows.Data;

namespace CanvasEditor
{
	public class EditableFrameworkElement : IEditable
	{

		internal EditableFrameworkElement(CanvasEditor editor, FrameworkElement element)
		{
			Editor = editor;
			UIElement = element;
			Position = new PointNotifyPropertyChanged(new Point(
				(double)element.GetValue(Canvas.LeftProperty),
				(double)element.GetValue(Canvas.TopProperty)));
			Size = new SizeNotifyPropertyChanged(new Size(
				(double)element.GetValue(Canvas.WidthProperty),
				(double)element.GetValue(Canvas.HeightProperty)));

			Binding xBinding = new Binding(PointNotifyPropertyChanged.XProperty);
			xBinding.Source = Position;
			element.SetBinding(Canvas.LeftProperty, xBinding);

			Binding yBinding = new Binding(PointNotifyPropertyChanged.YProperty);
			yBinding.Source = Position;
			element.SetBinding(Canvas.TopProperty, yBinding);

			Binding wBinding = new Binding(SizeNotifyPropertyChanged.WidthProperty);
			wBinding.Source = Size;
			element.SetBinding(Canvas.WidthProperty, wBinding);

			Binding hBinding = new Binding(SizeNotifyPropertyChanged.HeightProperty);
			hBinding.Source = Size;
			element.SetBinding(Canvas.HeightProperty, hBinding);

			Position.PropertyChanged += new System.ComponentModel.PropertyChangedEventHandler(Position_PropertyChanged);
			Size.PropertyChanged += new System.ComponentModel.PropertyChangedEventHandler(Size_PropertyChanged);
		}

		public CanvasEditor Editor { get; private set; }

		public UIElement UIElement { get; private set; }

		private FrameworkElement FrameworkElement
		{
			get
			{
				return (FrameworkElement)UIElement;
			}
		}

		public bool IsEnabled { get; set; }

		private EditableHandleManager HandleManager { get; set; }

		public bool Select {
			set
			{
				HandleManager.AlteringObjectsVisible = value;
			}
		}

		public PointNotifyPropertyChanged Position { get; private set; }

		public SizeNotifyPropertyChanged Size { get; private set; }

		public event OnEditablePositionChanged PositionChanged;

		public event OnEditableSizeChanged SizeChanged;

		void Size_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
		{
			if (PositionChanged != null)
			{
				PositionChanged(this, new System.EventArgs());
			}
		}

		void Position_PropertyChanged(object sender, System.ComponentModel.PropertyChangedEventArgs e)
		{
			if (SizeChanged != null)
			{
				SizeChanged(this, new System.EventArgs());
			}
		}


	}
}
