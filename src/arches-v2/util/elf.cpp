#include "elf.hpp"

#include "endian.hpp"
#include "file.hpp"

namespace Arches {

ELF::ELF_Header::ELF_Header(Util::File* file) {
	//Identifier
	file->read_bin(reinterpret_cast<uint8_t*>(&e_ident),sizeof(E_IDENT));

	#ifdef BUILD_ARCH_ENDIAN_LITTLE
	if (e_ident.magic.u32==0x464C457Fu); else throw ErrInvFile("magic number check failed");
	#else
	if (e_ident.magic.u32==0x7F454C46u); else throw ErrInvFile("magic number check failed");
	#endif

	switch (e_ident.ei_class) {
		case E_IDENT::EI_CLASS::ELFCLASS32: break;
		case E_IDENT::EI_CLASS::ELFCLASS64: break;
		default: throw ErrInvFile("invalid bit width flag "+std::to_string(static_cast<uint32_t>(e_ident.ei_class)));
	}

	switch (e_ident.ei_data) {
		case E_IDENT::EI_DATA::ELFDATA2LSB: break;
		case E_IDENT::EI_DATA::ELFDATA2MSB: break;
		default: throw ErrInvFile("invalid endianness flag "+std::to_string(static_cast<uint32_t>(e_ident.ei_data)));
	}

	switch (e_ident.ei_version) {
		case E_IDENT::EI_VERSION::EV_NONE: throw ErrInvFile("invalid version 0");
		case E_IDENT::EI_VERSION::EV_CURRENT: break;
		default: throw ErrUnaccFile("unknown future version "+std::to_string(static_cast<uint32_t>(e_ident.ei_version)));
	}

	e_type = static_cast<E_TYPE>(fix_endianness(file->read_bin<uint16_t>()));
	switch (e_type) {
		case E_TYPE::ET_NONE:   //fallthrough
		case E_TYPE::ET_REL:    //fallthrough
		case E_TYPE::ET_EXEC:   //fallthrough
		case E_TYPE::ET_DYN:    //fallthrough
		case E_TYPE::ET_CORE:   //fallthrough
		case E_TYPE::ET_LOOS:   //fallthrough
		case E_TYPE::ET_HIOS:   //fallthrough
		case E_TYPE::ET_LOPROC: //fallthrough
		case E_TYPE::ET_HIPROC: break;
		default: throw ErrInvFile("invalid file type flag "+std::to_string(static_cast<uint32_t>(e_type)));
	}

	e_machine = static_cast<E_MACHINE>(fix_endianness(file->read_bin<uint16_t>()));
	//TODO: check is reasonable

	e_version = static_cast<E_VERSION>(fix_endianness(file->read_bin<uint32_t>()));
	switch (e_version) {
		case E_VERSION::EV_NONE: throw ErrInvFile("invalid version 0");
		case E_VERSION::EV_CURRENT: break;
		default: throw ErrUnaccFile("unknown future version "+std::to_string(static_cast<uint32_t>(e_version)));
	}

	if (e_ident.ei_class==E_IDENT::EI_CLASS::ELFCLASS32) {
		e_entry.u32 = fix_endianness(file->read_bin<uint32_t>());
		e_phoff.u32 = fix_endianness(file->read_bin<uint32_t>());
		e_shoff.u32 = fix_endianness(file->read_bin<uint32_t>());
	} else {
		e_entry.u64 = fix_endianness(file->read_bin<uint64_t>());
		e_phoff.u64 = fix_endianness(file->read_bin<uint64_t>());
		e_shoff.u64 = fix_endianness(file->read_bin<uint64_t>());
	}

	e_flags = fix_endianness(file->read_bin<uint32_t>());

	e_ehsize = fix_endianness(file->read_bin<uint16_t>());
	if (
		e_ehsize ==
		( (e_ident.ei_class==E_IDENT::EI_CLASS::ELFCLASS32) ? 52 : 64 )
	); else throw ErrInvFile("inconsistent header size");

	e_phentsize = fix_endianness(file->read_bin<uint16_t>());
	e_phnum     = fix_endianness(file->read_bin<uint16_t>());

	e_shentsize = fix_endianness(file->read_bin<uint16_t>());
	e_shnum     = fix_endianness(file->read_bin<uint16_t>());

	e_shstrndx = fix_endianness(file->read_bin<uint16_t>());
}

template <typename RET> RET ELF::ELF_Header::fix_endianness(RET x) const {
	#ifdef BUILD_ARCH_ENDIAN_LITTLE
		if (e_ident.ei_data!=E_IDENT::EI_DATA::ELFDATA2LSB) return Util::Endian::reverse(x);
	#else
		if (e_ident.ei_data!=E_IDENT::EI_DATA::ELFDATA2MSB) return Util::Endian::reverse(x);
	#endif
	return x;
}


ELF::ProgramHeader::ArrayElement::ArrayElement(Util::File* file, ELF_Header const* elf_header) :
	_header(elf_header)
{
	//TODO: check loaded values are reasonable!
	p_type = static_cast<P_TYPE>(elf_header->fix_endianness(file->read_bin<uint32_t>()));

	//Note different order of fields.
	if (elf_header->e_ident.ei_class==ELF_Header::E_IDENT::EI_CLASS::ELFCLASS32) {
		p_offset.u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		p_vaddr. u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		p_paddr. u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		p_filesz.u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		p_memsz. u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		p_flags      = static_cast<P_FLAGS>(elf_header->fix_endianness(file->read_bin<uint32_t>()));
		p_align. u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
	} else {
		assert(elf_header->e_ident.ei_class==ELF_Header::E_IDENT::EI_CLASS::ELFCLASS64);
		p_flags      = static_cast<P_FLAGS>(elf_header->fix_endianness(file->read_bin<uint32_t>()));
		p_offset.u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		p_vaddr. u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		p_paddr. u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		p_filesz.u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		p_memsz. u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		p_align. u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
	}
}


ELF::ProgramHeader::ProgramHeader(Util::File* file, ELF_Header const* elf_header) {
	if (elf_header->e_phnum > 0) {
		arr.resize(elf_header->e_phnum);
		fseek(file->backing, elf_header->e_phoff.u64, SEEK_SET);
		for (size_t i=0;i<elf_header->e_phnum;++i) {
			arr[i] = ArrayElement(file,elf_header);
		}
	}
}

ELF::SectionHeader::ArrayElement::ArrayElement(Util::File* file, ELF_Header const* elf_header) :
	_header(elf_header)
{
	//TODO: check loaded values are reasonable!

	if(elf_header->e_ident.ei_class == ELF_Header::E_IDENT::EI_CLASS::ELFCLASS32) {
		sh_name          = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_type          = static_cast<SH_TYPE>(elf_header->fix_endianness(file->read_bin<uint32_t>()));
		sh_flags.    u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_addr.     u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_offset.   u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_size.     u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_link          = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_info          = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_addralign.u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_entsize.  u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
	}
	else {
		assert(elf_header->e_ident.ei_class == ELF_Header::E_IDENT::EI_CLASS::ELFCLASS64);
		sh_name          = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_type          = static_cast<SH_TYPE>(elf_header->fix_endianness(file->read_bin<uint32_t>()));
		sh_flags.    u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		sh_addr.     u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		sh_offset.   u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		sh_size.     u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		sh_link          = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_info          = elf_header->fix_endianness(file->read_bin<uint32_t>());
		sh_addralign.u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		sh_entsize.  u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
	}
}

ELF::SectionHeader::SectionHeader(Util::File* file, ELF_Header const* elf_header)
{
	if(elf_header->e_shnum > 0) {
		arr.resize(elf_header->e_shnum);
		fseek(file->backing, elf_header->e_shoff.u64, SEEK_SET);
		for(size_t i = 0; i < elf_header->e_shnum; ++i) {
			arr[i] = ArrayElement(file, elf_header);
		}
	}
}

ELF::SymbolTable::ArrayElement::ArrayElement(Util::File* file, ELF_Header const* elf_header) :
	_header(elf_header)
{
	//TODO: check loaded values are reasonable!
	if(elf_header->e_ident.ei_class == ELF_Header::E_IDENT::EI_CLASS::ELFCLASS32) {
		st_name      = elf_header->fix_endianness(file->read_bin<uint32_t>());
		st_value.u32 = elf_header->fix_endianness(file->read_bin<uint32_t>());
		st_size.u32  = elf_header->fix_endianness(file->read_bin<uint32_t>());
		st_info      = elf_header->fix_endianness(file->read_bin<uint8_t>());
		st_other     = elf_header->fix_endianness(file->read_bin<uint8_t>());
		st_shndx     = elf_header->fix_endianness(file->read_bin<uint16_t>());
	}
	else {
		assert(elf_header->e_ident.ei_class == ELF_Header::E_IDENT::EI_CLASS::ELFCLASS64);
		st_name      = elf_header->fix_endianness(file->read_bin<uint32_t>());
		st_value.u64 = elf_header->fix_endianness(file->read_bin<uint64_t>());
		st_size.u64  = elf_header->fix_endianness(file->read_bin<uint64_t>());
		st_info      = elf_header->fix_endianness(file->read_bin<uint8_t>());
		st_other     = elf_header->fix_endianness(file->read_bin<uint8_t>());
		st_shndx     = elf_header->fix_endianness(file->read_bin<uint16_t>());
	}
}

ELF::SymbolTable::SymbolTable(Util::File* file, const ELF_Header* elf_header, const SectionHeader::ArrayElement& section)
{

}

ELF::ELF(std::string const& path) {
	Util::File file(path,Util::File::MODE::R);

	elf_header     = nullptr;
	program_header = nullptr;
	section_header = nullptr;
	try {
		elf_header     = _new ELF_Header   (&file           );
		program_header = _new ProgramHeader(&file,elf_header);
		section_header = _new SectionHeader(&file, elf_header);

		for (ProgramHeader::ArrayElement& elem : program_header->arr) {
			if (elem.p_type==ProgramHeader::ArrayElement::P_TYPE::PT_LOAD) {
				LoadableSegment* seg = _new LoadableSegment(elf_header);
				elem.segment = seg;
				segments.emplace_back(seg);

				if (elf_header->e_ident.ei_class==ELF_Header::E_IDENT::EI_CLASS::ELFCLASS32) {
					seg->vaddr = elem.p_vaddr.u32;

					notimpl; //TODO: this!
				} else {
					seg->vaddr = elem.p_vaddr.u64;

					#ifdef BUILD_ARCH_32
					//If fails, this segment is larger than 4GiB and as-such cannot be loaded.
					assert(static_cast<uint64_t>(static_cast<size_t>(elem.p_memsz.u64))==elem.p_memsz.u64);
					#endif

					seg->data.resize(static_cast<size_t>(elem.p_memsz.u64));
					//memset(seg.data.data(),0x00,seg.data.size(); //unnecessary; `.resize(...)` fills with zeroes.

					fseek(file.backing,static_cast<long int>(elem.p_offset.u64),SEEK_SET);
					file.read_bin(seg->data.data(),static_cast<size_t>(elem.p_filesz.u64));
				}
			} else {
				elem.segment = nullptr;
			}
		}
	} catch (...) {
		delete program_header;
		delete elf_header;
		delete section_header;
		throw;
	}
}
ELF::~ELF() {
	for (LoadableSegment* seg : segments) {
		delete seg;
	}
	delete program_header;
	delete elf_header;
	delete section_header;
}


}
