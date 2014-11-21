using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml;

namespace ArmController
{
    public class Script
    {
        public List<List<Command>> Commands { get; private set; }

        public Script()
        {
            Commands = new List<List<Command>>();
        }

        public void AddCommand(List<Command> command)
        {
            Commands.Add(command);
        }

        public void Clear()
        {
            Commands.Clear();
        }

        public void Save(string filename)
        {
            System.Xml.Serialization.XmlSerializer s = new System.Xml.Serialization.XmlSerializer(this.GetType());
            using (XmlTextWriter writer = new XmlTextWriter(filename, null))
            {
                writer.Formatting = Formatting.Indented;
                s.Serialize(writer, this);
                writer.Close();
            }
        }

        public bool LoadScript(string filename)
        {
            System.Xml.Serialization.XmlSerializer s = new System.Xml.Serialization.XmlSerializer(this.GetType());

            if (File.Exists(filename))
            {
                var stream = new FileStream(filename, FileMode.Open, FileAccess.Read);

                using (XmlTextReader reader = new XmlTextReader(stream))
                {
                    var newScript = (Script)s.Deserialize(reader);
                    if (newScript != null)
                    {
                        this.Clear();
                        this.Commands = newScript.Commands;
                    }
                }
                return true;
            }

            else
            {
                return false;
            }
        }
    }

    public class Command
    {
        public int Id { get; set; }
        public int GoalPosition { get; set; }

        public Command()
        {
        }

        public Command(int id, int goalPosition)
        {
            this.Id = id;
            this.GoalPosition = goalPosition;
        }
    }
}
