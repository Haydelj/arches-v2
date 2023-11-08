#pragma once

#include "stdafx.hpp"

namespace Arches {


namespace Util {
	class File;
}


class ErrInvFile final : public std::runtime_error {
	public:
		explicit ErrInvFile(std::string const& msg) : std::runtime_error("Invalid file: "+msg+"!") {}
		virtual ~ErrInvFile() = default;
};
class ErrUnaccFile final : public std::runtime_error {
	public:
		explicit ErrUnaccFile(std::string const& msg) : std::runtime_error("Unacceptable file: "+msg+"!") {}
		virtual ~ErrUnaccFile() = default;
};


//The names in this class come from the actual ELF documentation:
//	http://man7.org/linux/man-pages/man5/elf.5.html
//	Note: different resources separate out the magic number from the identifier, making the relation
//		to the actual standard tricky.
class ELF final 
{
	public:
		//ELF header: table of data at file offset 0.
		//	Ref.: http://refspecs.linuxbase.org/elf/gabi4+/ch4.eheader.html
		class ELF_Header final {
			public:
				//Identifier
				class E_IDENT final { public:
					//Magic number: { 0x7F 'E' 'L' 'F' }
					union {
						uint8_t u8[4];
						uint32_t u32;
					} magic;

					//Bit width of target system
					enum class EI_CLASS : uint8_t {
						ELFCLASSNONE = 0, //Invalid
						ELFCLASS32   = 1, //32-bit architecture
						ELFCLASS64   = 2, //64-bit architecture
					} ei_class;

					//Endianness
					enum class EI_DATA : uint8_t {
						ELFDATANONE = 0, //Unknown data format
						ELFDATA2LSB = 1, //Two's complement, little-endian
						ELFDATA2MSB = 2  //Two's complement, big-endian
					} ei_data;

					//Version number of ELF specification
					enum class EI_VERSION : uint8_t {
						EV_NONE    = 0, //Invalid version
						EV_CURRENT = 1  //Current version
					} ei_version;

					//OS and ABI of target system
					//	Note: platform-specific values are keyed from this value.
					enum class EI_OSABI : uint8_t {
						ELFOSABI_NONE       =   0, //No extensions or unspecified (same as `ELFOSABI_SYSV`)
						ELFOSABI_SYSV       =   0, //UNIX System V
						ELFOSABI_HPUX       =   1, //Hewlett-Packard HP-UX
						ELFOSABI_NETBSD     =   2, //NetBSD
						ELFOSABI_LINUX      =   3, //Linux
						ELFOSABI_SOLARIS    =   6, //Sun Solaris
						ELFOSABI_AIX        =   7, //AIX
						ELFOSABI_IRIX       =   8, //IRIX
						ELFOSABI_FREEBSD    =   9, //FreeBSD
						ELFOSABI_TRU64      =  10, //Compaq TRU64 UNIX
						ELFOSABI_MODESTO    =  11, //Novell Modesto
						ELFOSABI_OPENBSD    =  12, //Open BSD
						ELFOSABI_OPENVMS    =  13, //Open VMS
						ELFOSABI_NSK        =  14, //Hewlett-Packard Non-Stop Kernel
						ELFOSABI_ARM        =  97, //ARM
						ELFOSABI_STANDALONE = 255  //Stand-alone (embedded)
					} ei_osabi;

					//Used to distinguish among incompatible versions of an ABI.  Often `0x00`:
					//	unspecified.
					uint8_t ei_abiversion;

					//Unused space.  Ought to be zeroes, but if not it's officially ignored.
					uint8_t ei_pad[7];
				} e_ident;
				static_assert(sizeof(E_IDENT)==16,"Implementation error!");

				//Object file type
				enum class E_TYPE : uint16_t {
					ET_NONE   = 0x0000, //No file type
					ET_REL    = 0x0001, //Relocatable file
					ET_EXEC   = 0x0002, //Executable file
					ET_DYN    = 0x0003, //Shared object file
					ET_CORE   = 0x0004, //Core file
					ET_LOOS   = 0xFE00, //OS-specific
					ET_HIOS   = 0xFEFF, //OS-specific
					ET_LOPROC = 0xFF00, //Processor-specific
					ET_HIPROC = 0xFFFF  //Processor-specific
				} e_type;

				//Required architecture
				enum class E_MACHINE : uint16_t {
					EM_NONE        =   0, //Unknown / no machine
					EM_M32         =   1, //AT&T WE 32100
					EM_SPARC       =   2, //Sun Microsystems SPARC
					EM_386         =   3, //Intel 80386
					EM_68K         =   4, //Motorola 68000
					EM_88K         =   5, //Motorola 88000
					//(6 reserved; was `EM_486`)
					EM_860         =   7, //Intel 80860
					EM_MIPS        =   8, //MIPS I RS3000 (big-endian only) architecture
					EM_S370        =   9, //IBM System/370 Processor
					EM_MIPS_RS3_LE =  10, //MIPS RS3000 Little-endian
					//([11-14] reserved)
					EM_PARISC      =  15, //Hewlett-Packard PA-RISC
					//(16 reserved)
					EM_VPP500      =  17, //Fujitsu VPP500
					EM_SPARC32PLUS =  18, //Enhanced instruction set SPARC
					EM_960         =  19, //Intel 80960
					EM_PPC         =  20, //PowerPC
					EM_PPC64       =  21, //PowerPC 64-bit
					EM_S390        =  22, //IBM System/390 Processor
					//([23-35] reserved)
					EM_V800        =  36, //NEC V800
					EM_FR20        =  37, //Fujitsu FR20
					EM_RH32        =  38, //TRW RH-32
					EM_RCE         =  39, //Motorola RCE
					EM_ARM         =  40, //Advanced RISC Machines ARM
					EM_ALPHA       =  41, //Digital Alpha
					EM_SH          =  42, //Renesas SuperH / Hitachi SH
					EM_SPARCV9     =  43, //SPARC Version 9 64-bit
					EM_TRICORE     =  44, //Siemens TriCore embedded processor
					EM_ARC         =  45, //Argonaut RISC Core, Argonaut Technologies Inc.
					EM_H8_300      =  46, //Hitachi H8/300
					EM_H8_300H     =  47, //Hitachi H8/300H
					EM_H8S         =  48, //Hitachi H8S
					EM_H8_500      =  49, //Hitachi H8/500
					EM_IA_64       =  50, //Intel Itanium IA-64 processor architecture
					EM_MIPS_X      =  51, //Stanford MIPS-X
					EM_COLDFIRE    =  52, //Motorola ColdFire
					EM_68HC12      =  53, //Motorola M68HC12
					EM_MMA         =  54, //Fujitsu MMA Multimedia Accelerator
					EM_PCP         =  55, //Siemens PCP
					EM_NCPU        =  56, //Sony nCPU embedded RISC processor
					EM_NDR1        =  57, //Denso NDR1 microprocessor
					EM_STARCORE    =  58, //Motorola Star*Core processor
					EM_ME16        =  59, //Toyota ME16 processor
					EM_ST100       =  60, //STMicroelectronics ST100 processor
					EM_TINYJ       =  61, //Advanced Logic Corp. TinyJ embedded processor family
					EM_X86_64      =  62, //AMD x86-64 architecture
					EM_PDSP        =  63, //Sony DSP Processor
					EM_PDP10       =  64, //Digital Equipment Corp. PDP-10
					EM_PDP11       =  65, //Digital Equipment Corp. PDP-11
					EM_FX66        =  66, //Siemens FX66 microcontroller
					EM_ST9PLUS     =  67, //STMicroelectronics ST9+ 8/16 bit microcontroller
					EM_ST7         =  68, //STMicroelectronics ST7 8-bit microcontroller
					EM_68HC16      =  69, //Motorola MC68HC16 Microcontroller
					EM_68HC11      =  70, //Motorola MC68HC11 Microcontroller
					EM_68HC08      =  71, //Motorola MC68HC08 Microcontroller
					EM_68HC05      =  72, //Motorola MC68HC05 Microcontroller
					EM_SVX         =  73, //Silicon Graphics SVx
					EM_ST19        =  74, //STMicroelectronics ST19 8-bit microcontroller
					EM_VAX         =  75, //DEC VAX
					EM_CRIS        =  76, //Axis Communications 32-bit embedded processor
					EM_JAVELIN     =  77, //Infineon Technologies 32-bit embedded processor
					EM_FIREPATH    =  78, //Element 14 64-bit DSP Processor
					EM_ZSP         =  79, //LSI Logic 16-bit DSP Processor
					EM_MMIX        =  80, //Donald Knuth's educational 64-bit processor
					EM_HUANY       =  81, //Harvard University machine-independent object files
					EM_PRISM       =  82, //SiTera Prism
					EM_AVR         =  83, //Atmel AVR 8-bit microcontroller
					EM_FR30        =  84, //Fujitsu FR30
					EM_D10V        =  85, //Mitsubishi D10V
					EM_D30V        =  86, //Mitsubishi D30V
					EM_V850        =  87, //NEC v850
					EM_M32R        =  88, //Mitsubishi M32R
					EM_MN10300     =  89, //Matsushita MN10300
					EM_MN10200     =  90, //Matsushita MN10200
					EM_PJ          =  91, //picoJava
					EM_OPENRISC    =  92, //OpenRISC 32-bit embedded processor
					EM_ARC_A5      =  93, //ARC Cores Tangent-A5
					EM_XTENSA      =  94, //Tensilica Xtensa Architecture
					EM_VIDEOCORE   =  95, //Alphamosaic VideoCore processor
					EM_TMM_GPP     =  96, //Thompson Multimedia General Purpose Processor
					EM_NS32K       =  97, //National Semiconductor 32000 series
					EM_TPC         =  98, //Tenor Network TPC processor
					EM_SNP1K       =  99, //Trebia SNP 1000 processor
					EM_ST200       = 100  //STMicroelectronics (www.st.com) ST200 microcontroller
				} e_machine;

				//File version
				enum class E_VERSION : uint32_t {
					EV_NONE    = 0u,
					EV_CURRENT = 1u
				} e_version;

				//Entry point (zero if none)
				union { uint32_t u32; uint64_t u64; } e_entry;

				//Program header table file offset (zero if none)
				union { uint32_t u32; uint64_t u64; } e_phoff;

				//Section header table file offset (zero if none)
				union { uint32_t u32; uint64_t u64; } e_shoff;

				//Processor-specific flags
				uint32_t e_flags;

				//Size of this header
				uint16_t e_ehsize;

				//Size of each entry in program header table
				uint16_t e_phentsize;
				//Number of entries in program header table (zero if no table)
				uint16_t e_phnum;

				//Size of each entry in section header table
				uint16_t e_shentsize;
				//Number of entries in section header table (zero if the number is
				//	larger than 0xFF00 (in which case the actual number is stored in `.sh_size` in
				//	the section header, which would otherwise be zero) or if no table)
				uint16_t e_shnum;

				//Section header table index of entry associated with the section name string table
				//	(zero if none)
				uint16_t e_shstrndx;

			public:
				explicit ELF_Header(Util::File* file);
				~ELF_Header() = default;

				template <typename RET> RET fix_endianness(RET x) const;
		};
		ELF_Header* elf_header;

		class LoadableSegment;

		//Program header: array of structures, each describing a segment (comprised of section(s))
		//	or other information the system needs to prepare the program for execution
		//	Ref.: http://refspecs.linuxbase.org/elf/gabi4+/ch5.pheader.html
		class ProgramHeader final {
			public:
				class ArrayElement final {
					private:
						ELF_Header const* _header;
					public:
						//Type of element
						enum class P_TYPE : uint32_t {
							//Array element is unused; other members' values undefined
							PT_NULL    = 0x00000000u,
							//Loadable segment of size `.p_memsz` initialized with data of length
							//	`.p_filesz` and the rest padded with zeroes
							PT_LOAD    = 0x00000001u,
							//Dynamic linking information
							PT_DYNAMIC = 0x00000002u,
							//Location and size of a null-terminated path name to invoke as an
							//	interpreter
							PT_INTERP  = 0x00000003u,
							//Location and size of auxiliary information
							PT_NOTE    = 0x00000004u,
							//(Reserved)
							PT_SHLIB   = 0x00000005u,
							//Location and size of the program header table
							PT_PHDR    = 0x00000006u,
							//Thread-local storage template
							PT_TLS     = 0x00000007u,
							//Range of OS-specific semantics
							PT_LOOS    = 0x60000000u,
							PT_HIOS    = 0x6FFFFFFFu,
							//Range of processor-specific semantics
							PT_LOPROC  = 0x70000000u,
							PT_HIPROC  = 0x7FFFFFFFu
						} p_type;

						//File offset of segment data
						union { uint32_t u32; uint64_t u64; } p_offset;

						//Virtual address where segment should be loaded
						union { uint32_t u32; uint64_t u64; } p_vaddr;

						//Physical address, as-relevant
						union { uint32_t u32; uint64_t u64; } p_paddr;

						//Size of the segment data contained in the file (may be zero)
						union { uint32_t u32; uint64_t u64; } p_filesz;

						//Size of the segment (may be zero, but probably shouldn't be)
						union { uint32_t u32; uint64_t u64; } p_memsz;

						enum class P_FLAGS : uint32_t {
							PF_X        = 0x00000001u, //Execute
							PF_W        = 0x00000002u, //Write
							PF_R        = 0x00000004u, //Read
							PF_MASKOS   = 0x0FF00000u, //Unspecified
							PF_MASKPROC = 0xF0000000u  //Unspecified
						} p_flags;

						//If greater than 1, value is the alignment of the segment in memory and in
						//	the file data (i.e., it evenly divides `.p_offset` and `.p_vaddr`).
						union { uint32_t u32; uint64_t u64; } p_align;

						//If a loadable segment, updated to point to the segment's data (note:
						//	updated to `nullptr` if not a loadable segment).
						LoadableSegment* segment;

					public:
						ArrayElement() = default;
						ArrayElement(Util::File* file, ELF_Header const* elf_header);
						~ArrayElement() = default;
				};

				std::vector<ArrayElement> arr;

			public:
				ProgramHeader(Util::File* file, ELF_Header const* elf_header);
				~ProgramHeader() = default;
		};
		ProgramHeader* program_header;

		class SectionHeader final {
		public:
			class ArrayElement final {
			private:
				ELF_Header const* _header;
			public:
				//Type of element
				uint32_t sh_name;

				enum class SH_TYPE : uint32_t {
					SHT_NULL = 0x0,
					SHT_PROGBITS = 0x1,
					SHT_SYMTAB = 0x2,
					SHT_STRTAB = 0x3,
					SHT_RELA = 0x4,
					SHT_HASH = 0x5,
					SHT_DYNAMIC = 0x6,
					SHT_NOTE = 0x7,
					SHT_NOBITS = 0x8,
					SHT_REL = 0x9,
					SHT_SHLIB = 0xa,
					SHT_DYNSYM = 0xb,
					SHT_INIT_ARRAY = 0xe,
					SHT_FINI_ARRAY = 0xf,
					SHT_PREINIT_ARRAY = 0x10,
					SHT_GROUP = 0x11,
					SHT_SYMTAB_SHNDX = 0x12,
					SHT_LOPROC = 0x70000000,
					SHT_HIPROC = 0x7fffffff,
					SHT_LOUSER = 0x80000000,
					SHT_HIUSER = 0xffffffff,
				} sh_type;

				union { uint32_t u32; uint64_t u64; } sh_flags;
				union { uint32_t u32; uint64_t u64; } sh_addr;
				union { uint32_t u32; uint64_t u64; } sh_offset;
				union { uint32_t u32; uint64_t u64; } sh_size;
				uint32_t sh_link;
				uint32_t sh_info;
				union { uint32_t u32; uint64_t u64; } sh_addralign;
				union { uint32_t u32; uint64_t u64; } sh_entsize;

			public:
				ArrayElement() = default;
				ArrayElement(Util::File* file, ELF_Header const* elf_header);
				~ArrayElement() = default;
			};

			std::vector<ArrayElement> arr;

		public:
			SectionHeader(Util::File* file, ELF_Header const* elf_header);
			~SectionHeader() = default;
		};
		SectionHeader* section_header;

		class SymbolTable final {
		public:
			class ArrayElement final {
			private:
				ELF_Header const* _header;
			public:
				//Type of element
				uint32_t st_name;
				uint8_t st_info;
				uint8_t st_other;
				uint16_t st_shndx;
				union { uint32_t u32; uint64_t u64; } st_value;
				union { uint32_t u32; uint64_t u64; } st_size;

			public:
				ArrayElement() = default;
				ArrayElement(Util::File* file, ELF_Header const* elf_header);
				~ArrayElement() = default;
			};

			std::vector<ArrayElement> arr;

		public:
			SymbolTable(Util::File* file, const ELF_Header* elf_header, const SectionHeader::ArrayElement& section);
			~SymbolTable() = default;
		};
		SymbolTable* symbol_table;

		//Loadable segments
		class LoadableSegment final {
			private:
				ELF_Header const* _header;
			public:
				//Virtual address where segment should be loaded
				vaddr_t vaddr;

				//Data loaded into the start of the segment
				std::vector<uint8_t> data;

			public:
				explicit LoadableSegment(ELF_Header const* header) : _header(header) {}
				~LoadableSegment() = default;
		};
		std::vector<LoadableSegment*> segments;

	public:
		explicit ELF(std::string const& path);
		~ELF();
};


}
