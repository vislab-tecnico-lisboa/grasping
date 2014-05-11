#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class CVSRow
{
    public:
        std::string const& operator[](std::size_t index) const
        {
            return m_data[index];
        }
        std::size_t size() const
        {
            return m_data.size();
        }
        void readNextRow(std::istream& str)
        {
            std::string         line;
            std::getline(str,line);

            std::stringstream   lineStream(line);
            std::string         cell;

            m_data.clear();
            while(std::getline(lineStream,cell,','))
            {
                m_data.push_back(cell);
            }
        }
    private:
        std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str,CVSRow& data)
{
    data.readNextRow(str);
    return str;
}

class CVSIterator
{
    public:
        typedef std::input_iterator_tag     iterator_category;
        typedef CVSRow                      value_type;
        typedef std::size_t                 difference_type;
        typedef CVSRow*                     pointer;
        typedef CVSRow&                     reference;

        CVSIterator(std::istream& str)  :m_str(str.good()?&str:NULL) { ++(*this); }
        CVSIterator()                   :m_str(NULL) {}

        // Pre Increment
        CVSIterator& operator++()               {if (m_str) { (*m_str) >> m_row;m_str = m_str->good()?m_str:NULL;}return *this;}
        // Post increment
        CVSIterator operator++(int)             {CVSIterator    tmp(*this);++(*this);return tmp;}
        CVSRow const& operator*()   const       {return m_row;}
        CVSRow const* operator->()  const       {return &m_row;}

        bool operator==(CVSIterator const& rhs) {return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));}
        bool operator!=(CVSIterator const& rhs) {return !((*this) == rhs);}
    private:
        std::istream*       m_str;
        CVSRow              m_row;
};

class HumanGraspsParser
{
	public:
		HumanGraspsParser()
		{};

		std::vector<std::vector<double> > parse(std::string & file_name)
		{
			std::vector<std::vector<double> > grasps;
			std::ifstream file(file_name.c_str());
			if(!file.is_open())
			{
				std::cout << "Problem opening file: " << file_name << std::endl;
				exit(-1);
			}
			if(!file.good())
			{
				std::cout << "Problem with file: " << file_name << std::endl;
				exit(-1);
			}
			CVSIterator line(file);
			++line;
			for(;line != CVSIterator();++line)
			{
				std::vector<double> human_grasp=boost::assign::list_of
				(atof((*line)[2].c_str()))
				(atof((*line)[4].c_str()))
				(atof((*line)[6].c_str()))
				(atof((*line)[8].c_str()))
				(atof((*line)[10].c_str()))
				(atof((*line)[11].c_str()));

				grasps.push_back(human_grasp);
			}
			std::cout << "Parsed human grasps knowledge:" << grasps.size() << " " <<grasps[0].size() << std::endl;

//			for(unsigned int i=0; i< grasps.size();++i)
//			{
//				for(unsigned int j=0; j< grasps[i].size();++j)
//				{
//					std::cout << grasps[i][j] << " ";
//				}
//				std::cout << std::endl;
//			}

			return grasps;
		}

};


class GraspsKernelParser
{
	public:
		GraspsKernelParser()
		{};

		std::vector<std::vector<double> > parse(std::string & file_name)
		{
			std::vector<std::vector<double> > kernel;
			std::ifstream file(file_name.c_str());
			if(!file.is_open())
			{
				std::cout << "Problem opening file: " << file_name << std::endl;
				exit(-1);
			}
			if(!file.good())
			{
				std::cout << "Problem with file: " << file_name << std::endl;
				exit(-1);
			}
			CVSIterator line(file);
			++line;
			for(;line != CVSIterator();++line)
			{
				std::vector<double> kernel_line;
				for(unsigned int i=0; i < (*line).size(); ++i)
				{
					kernel_line.push_back((double)atof((*line)[i].c_str()));
				}


				kernel.push_back(kernel_line);
			}

			std::cout << "Parsed similar grasps kernel:" << kernel.size() << " " <<kernel[0].size() << std::endl;
//			for(unsigned int i=0; i< kernel.size();++i)
//			{
//				for(unsigned int j=0; j< kernel[i].size();++j)
//				{
//					std::cout << kernel[i][j] << " ";
//				}
//				std::cout << std::endl;
//			}

			return kernel;
		}

};
