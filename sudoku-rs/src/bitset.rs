use crate::consts;
use crate::consts::Consts;


type Num = u8;

pub struct Bitset<const N: usize> {
    bits: u16, // TODO: assert it fits
}

impl <const N: usize> Bitset<N>
    where () : Consts<N>
{
    const ALPHABET: [ char; N ] = <() as Consts<N>>::ALPHABET;

    pub fn make_empty() -> Self {
        Self { bits: 0 }
    }

    pub fn make_solved(num: Num) -> Self {
        assert!((num as usize) < N);
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
        assert!(self.bits != 0);
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

    pub fn test(&self, num: Num) -> bool {
        assert!((num as usize) < N);
        (self.bits & (1 << num)) != 0
    }

    pub fn to_printable_char(&self) -> char {
        let ones = self.bits.count_ones() as usize;
        assert!(ones > 0);
        assert!(ones <= N);
        match ones {
            0 => panic!("Invalid Bitset: no bits set"),
            1 => Self::ALPHABET[self.bits.trailing_zeros() as usize],
            n if n == N => consts::UNSOLVED,
            _ => consts::PARTIAL
        }
    }

    pub fn from_printable_char(c: char) -> Self {
        match c {
            consts::UNSOLVED => return Self::make_full(),
            _ => {
                let pos = Self::ALPHABET.iter().position(|&x| x == c);
                match pos {
                    Some(i) => Self::make_solved(i as Num),
                    None => panic!("Invalid character for Bitset: {}", c)
                }
            }
        }
    }
}


#[cfg(test)]
mod tests {
use super::Bitset;


#[test]
fn test_bitset()
{
    {
        let bs = Bitset::<4>::make_empty();
        assert_eq!(bs.count_ones(), 0);
        assert_eq!(bs.to_ulong(), 0);
    }

    {
        let bs = Bitset::<4>::make_full();
        assert_eq!(bs.count_ones(), 4);
        assert_eq!(bs.to_ulong(), 0b1111);
    }

    {
        let bs = Bitset::<4>::make_solved(0);
        assert_eq!(bs.count_ones(), 1);
        assert_eq!(bs.to_ulong(), 0b0001);
    }

    {
        let bs = Bitset::<4>::make_solved(1);
        assert_eq!(bs.count_ones(), 1);
        assert_eq!(bs.to_ulong(), 0b0010);
    }

    {
        let bs = Bitset::<4>::make_solved(2);
        assert_eq!(bs.count_ones(), 1);
        assert_eq!(bs.to_ulong(), 0b0100);
    }

    {
        let bs = Bitset::<4>::make_solved(3);
        assert_eq!(bs.count_ones(), 1);
        assert_eq!(bs.to_ulong(), 0b1000);
    }
}


#[test]
fn test_bitset_exclude()
{
    let mut bs = Bitset::<4>::make_full();
    assert_eq!(bs.to_ulong(), 0b1111);

    //
    let bs1 = Bitset::<4>::make_from_bits(0b0011);
    assert_eq!(bs1.to_ulong(), 0b0011);

    assert_eq!(bs.exclude(&bs1), true);
    assert_eq!(bs.to_ulong(), 0b1100);
    assert_eq!(bs.exclude(&bs1), false);
    assert_eq!(bs.to_ulong(), 0b1100);

    //
    assert_eq!(bs.to_ulong(), 0b1100);

    let bs2 = Bitset::<4>::make_from_bits(0b0110);
    assert_eq!(bs2.to_ulong(), 0b0110);

    assert_eq!(bs.exclude(&bs2), true);
    assert_eq!(bs.to_ulong(), 0b1000);
    assert_eq!(bs.exclude(&bs2), false);
    assert_eq!(bs.to_ulong(), 0b1000);
}


#[test]
fn test_bitset_to_printable_char()
{
    let bs = Bitset::<9>::make_solved(0);
    assert_eq!(bs.to_printable_char(), '1');

    let bs = Bitset::<9>::make_solved(8);
    assert_eq!(bs.to_printable_char(), '9');

    let bs = Bitset::<9>::make_full();
    assert_eq!(bs.to_printable_char(), '*');

    let bs = Bitset::<9>::make_from_bits(0b100000111);
    assert_eq!(bs.to_printable_char(), '.');
}


#[test]
fn test_bitset_from_printable_char()
{
    let bs = Bitset::<9>::from_printable_char('1');
    assert_eq!(bs.to_ulong(), 0b000000001);

    let bs = Bitset::<9>::from_printable_char('5');
    assert_eq!(bs.to_ulong(), 0b000010000);

    let bs = Bitset::<9>::from_printable_char('9');
    assert_eq!(bs.to_ulong(), 0b100000000);

    let bs = Bitset::<9>::from_printable_char('*');
    assert_eq!(bs.to_ulong(), 0b111111111);
}


#[test]
fn test_bitset_printable_roundtrip()
{
    for n in 0..9 {
        let bs = Bitset::<9>::make_solved(n);
        let c = bs.to_printable_char();
        let bs2 = Bitset::<9>::from_printable_char(c);
        assert_eq!(bs.to_ulong(), bs2.to_ulong());
    }

    for c in Bitset::<9>::ALPHABET {
        let bs = Bitset::<9>::from_printable_char(c);
        let c2 = bs.to_printable_char();
        assert_eq!(c, c2);
    }
}


} // mod tests

