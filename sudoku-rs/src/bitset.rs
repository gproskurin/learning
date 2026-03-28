
type Num = u8;



pub struct Bitset<const N: u8> {
    bits: u16, // TODO: assert it fits
}

impl <const N: u8> Bitset<N> {
    pub fn make_empty() -> Self {
        Self { bits: 0 }
    }

    pub fn make_solved(num: Num) -> Self {
        Self { bits: 1 << num }
    }

    pub fn make_full() -> Self {
        Self { bits: (1 << N) - 1 }
    }

    pub fn make_from_bits(bits: u16) -> Self {
        assert!(bits != 0);
        assert!(bits < (1 << N));
        Self { bits }
    }

    pub fn is_solved(&self) -> bool {
        self.bits.count_ones() == 1
    }

    pub fn exclude(&mut self, excl: &Self) -> bool {
        let old = self.bits;
        self.bits &= !excl.bits;
        old != self.bits
    }

    pub fn get_solved_opt(&self) -> Option<Num> {
        if self.bits.count_ones() != 1 {
            return None;
        }
        Some(self.bits.trailing_zeros() as Num)
    }

    pub fn count_ones(&self) -> u32 {
        self.bits.count_ones()
    }

    pub fn to_ulong(&self) -> u16 {
        self.bits
    }
}

/*
template <num_t N>
bool bitset_exclude_set(bitset_t<N>& bs, const bitset_t<N>& excl)
{
	assert(bs.any());
	auto const old = bs;
	bs &= ~excl;
	assert(bs.any());
	auto const changed = old ^ bs;
	return changed.any();
}


template <num_t N>
std::optional<num_t> bitset_get_solved_opt(const bitset_t<N>& bs)
{
	assert(bs.any());
	if (bs.count() != 1) {
		return std::nullopt;
	}
	for (num_t n=0; n<N; ++n) {
		if (bs.test(n)) {
			return n;
		}
	}
	assert("unreacheable code");
	return std::nullopt; // FIXME
}


template <num_t N>
char bitset_parser_print(const bitset_t<N>& bs)
{
	assert(bs.any());
	switch (bs.count()) {
		case N:
			return '*';
		case 1:
			return num_print<N>(bitset_get_solved_opt<N>(bs).value());
		default:
			return '.';
	}
}
*/
