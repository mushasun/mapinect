using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.Diagnostics;

namespace PcdLoader
{
    public partial class Form1 : Form
    {
        public string Directory { get; set; }
        List<PcdFile> files;
        List<PcdFile> selectedFiles;
        public Form1()
        {
            InitializeComponent();
            Directory = Application.StartupPath;
            files = new List<PcdFile>();
            selectedFiles = new List<PcdFile>();
            dataGridView1.AutoGenerateColumns = false;
            SetGridLayout();
            LoadData();
            
        }

        private void LoadData()
        {
            DirectoryInfo dinfo = new DirectoryInfo(Directory);
            FileInfo[] Files = dinfo.GetFiles("*.pcd");
            foreach (FileInfo file in Files)
            {
                files.Add(new PcdFile(file.FullName, file.Name, file.LastWriteTime));
            }

            files = files.OrderByDescending(f => f.date).ToList();
            
            listBox1.DataSource = files;
            listBox1.DisplayMember = "name";
            dataGridView1.AutoResizeColumns();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            FolderBrowserDialog fbd = new FolderBrowserDialog();
            fbd.SelectedPath = Directory;
            fbd.ShowDialog();
            Directory = fbd.SelectedPath;

            LoadData();

        }

        private void listBox1_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            PcdFile pcdFile = (PcdFile)listBox1.SelectedItem;
            selectedFiles.Add(pcdFile);

            dataGridView1.DataSource = typeof(List<PcdFile>); 
            dataGridView1.DataSource = selectedFiles;
        }

        private void SetGridLayout()
        {

            //clear any previously set columns
            dataGridView1.Columns.Clear();

            DataGridViewTextBoxColumn dgridColID = new DataGridViewTextBoxColumn();
            dgridColID.HeaderText = "Name";
            dgridColID.Name = "name";
            dgridColID.Width = 280;
            dgridColID.DataPropertyName = "name";
            dataGridView1.Columns.Add(dgridColID);

            DataGridViewTextBoxColumn dgridColName = new DataGridViewTextBoxColumn();
            dgridColName.HeaderText = "Size";
            dgridColName.Name = "size";
            dgridColName.Width = 60;
            dgridColName.DataPropertyName = "size";
            dataGridView1.Columns.Add(dgridColName);

            DataGridViewTextBoxColumn dgridColDate = new DataGridViewTextBoxColumn();
            dgridColDate.HeaderText = "R";
            dgridColDate.Name = "R";
            dgridColDate.Width = 60;
            dgridColDate.DataPropertyName = "R";
            dataGridView1.Columns.Add(dgridColDate);

            DataGridViewTextBoxColumn dgridColDateG = new DataGridViewTextBoxColumn();
            dgridColDateG.HeaderText = "G";
            dgridColDateG.Name = "G";
            dgridColDateG.Width = 60;
            dgridColDateG.DataPropertyName = "G";
            dataGridView1.Columns.Add(dgridColDateG);

            DataGridViewTextBoxColumn dgridColDateB = new DataGridViewTextBoxColumn();
            dgridColDateB.HeaderText = "B";
            dgridColDateB.Name = "B";
            dgridColDateB.Width = 60;
            dgridColDateB.DataPropertyName = "B";
            dataGridView1.Columns.Add(dgridColDateB);


            DataGridViewTextBoxColumn dgridColPhone = new DataGridViewTextBoxColumn();
            dgridColPhone.HeaderText = "Opacity";
            dgridColPhone.Name = "opacity";
            dgridColPhone.Width = 60;
            dgridColPhone.DataPropertyName = "opacity";
            dataGridView1.Columns.Add(dgridColPhone);

        }

        private void button3_Click(object sender, EventArgs e)
        {
            StringBuilder sb = new StringBuilder();
            foreach (DataGridViewRow tt in dataGridView1.Rows)
            {
                PcdFile pcd = (PcdFile)tt.DataBoundItem;
                sb.Append(string.Format("\"{0}\" -ps {1} -fc {2},{3},{4} -opaque {5} ",
                                        pcd.fullName, 
                                        pcd.size,
                                        pcd.R,
                                        pcd.G,
                                        pcd.B,
                                        pcd.opacity));
            }

            if (chxShowAxis.Checked)
            {
                sb.Append(string.Format("-ax {0} -ax_pos {1},{2},{3}",
                                        txtAxisScale.Text, txtAxisX.Text, txtAxisY.Text, txtAxisZ.Text));
            }

            // Start the process.
            Process pcd_viewer = new Process();
            pcd_viewer.StartInfo.FileName = "pcd_viewer.exe";
            pcd_viewer.StartInfo.Arguments = sb.ToString();
            //pcd_viewer.StartInfo.UseShellExecute = false;
            //pcd_viewer.StartInfo.RedirectStandardOutput = true;
            pcd_viewer.Start();
        }

        private void button2_Click(object sender, EventArgs e)
        {
            foreach (var selected in listBox1.SelectedItems)
            {
                PcdFile pcdFile = (PcdFile)selected;
                selectedFiles.Add(pcdFile);
            }
            dataGridView1.DataSource = typeof(List<PcdFile>);
            dataGridView1.DataSource = selectedFiles;
        }

        private void dataGridView1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Delete)
            {
                foreach (DataGridViewRow selected in dataGridView1.SelectedRows)
                {
                    selectedFiles.Remove((PcdFile)selected.DataBoundItem);
                    //string name = selected.Cells[0].ToString();
                    //selectedFiles.Remove(selectedFiles.FirstOrDefault(f => f.name == name));
                }
                dataGridView1.DataSource = typeof(List<PcdFile>);
                dataGridView1.DataSource = selectedFiles;
            }
        }

        private void button4_Click(object sender, EventArgs e)
        {
            files = new List<PcdFile>();
            selectedFiles = new List<PcdFile>();
            dataGridView1.AutoGenerateColumns = false;
            SetGridLayout();
            LoadData();
        }

        private void Form1_KeyPress(object sender, KeyPressEventArgs e)
        {
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.F5)
            {
                files = new List<PcdFile>();
                selectedFiles = new List<PcdFile>();
                dataGridView1.AutoGenerateColumns = false;
                SetGridLayout();
                LoadData();
            }
        }


    }
}
