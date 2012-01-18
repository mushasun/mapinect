using System.Windows;
using PropertyChangedLibrary;

namespace CanvasEditor
{
	class HandleEditable : IEditableHandle
	{

		public HandleEditable(IEditable element, BoundaryPosition bPos)
		{
			EditableElement = element;
			BoundaryPosition = bPos;
			Position = BoundaryPositionToPoint();
			UIElement = HandleEditableCreator.CreateHandle(Position);
			element.SizeChanged += new OnEditableSizeChanged(element_SizeChanged);
		}

		public UIElement UIElement { get; set; }

		public IEditable EditableElement { get; private set; }

		public BoundaryPosition BoundaryPosition { get; private set; }

		private PointNotifyPropertyChanged BoundaryPositionToPoint()
		{
			Point point;
			Point position = EditableElement.Position.Point;
			Size size = EditableElement.Size.Size;
			switch (BoundaryPosition)
			{
				case BoundaryPosition.TopLeft:
					point = position;
					break;
				case BoundaryPosition.TopCenter:
					point = new Point(position.X + size.Width * .5, position.Y);
					break;
				case BoundaryPosition.TopRight:
					point = new Point(position.X + size.Width, position.Y);
					break;
				case BoundaryPosition.MidRight:
					point = new Point(position.X + size.Width, position.Y + size.Height * .5);
					break;
				case BoundaryPosition.BottomRight:
					point = new Point(position.X + size.Width, position.Y + size.Height);
					break;
				case BoundaryPosition.BottomCenter:
					point = new Point(position.X + size.Width * .5, position.Y + size.Height);
					break;
				case BoundaryPosition.BottomLeft:
					point = new Point(position.X, position.Y + size.Height);
					break;
				case BoundaryPosition.MidLeft:
				default:
					point = new Point(position.X, position.Y + size.Height * .5);
					break;
			}

			return new PointNotifyPropertyChanged(point);
		}

		public PointNotifyPropertyChanged Position { get; private set; }

		private bool myIsVisible;
		public bool IsVisible {
			get
			{
				return myIsVisible;
			}
			set
			{
				myIsVisible = value;
				HandleElement.Visibility = myIsVisible ? Visibility.Visible : Visibility.Hidden;
			}
		}

		public event OnHandlePositionChanged PositionChanged;

		private FrameworkElement HandleElement { get; set; }

		private void element_SizeChanged(object sender, EditableEventArgs e)
		{
			if (e.Editable == EditableElement)
			{
				
			}
		}

	}
}
