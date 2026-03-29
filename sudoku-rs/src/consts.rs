//

pub trait Consts<const N: usize> {
    const ALPHABET: [ char; N ];
}

impl Consts<9> for () {
    const ALPHABET: [ char; 9] = [ '1', '2', '3', '4', '5', '6', '7', '8', '9' ];
}
impl Consts<4> for () {
    const ALPHABET: [ char; 4] = [ '1', '2', '3', '4' ];
}


pub const PARTIAL: char = '.';
pub const UNSOLVED: char = '*';

