//

pub trait Alphabet<const N: usize> {
    const ALPHABET: [ char; N ];
}

impl Alphabet<9> for () {
    const ALPHABET: [ char; 9] = [ '1', '2', '3', '4', '5', '6', '7', '8', '9' ];
}
impl Alphabet<4> for () {
    const ALPHABET: [ char; 4] = [ '1', '2', '3', '4' ];
}


pub const PARTIAL: char = '.';
pub const UNSOLVED: char = '*';

