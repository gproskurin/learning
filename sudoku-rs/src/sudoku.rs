use crate::bitset;
use crate::consts;
use crate::consts::Consts;


type Num = u8;
const N: usize = 9;


type SudokuData<const N: usize> = [ [bitset::Bitset<N>; N]; N ];

pub struct Sudoku<const N: usize> {
    data: SudokuData<N>,
}

impl<const N: usize> Sudoku<N>
    where () : Consts<N>
{
    const ALPHABET: [ char; N ] = <() as Consts<N>>::ALPHABET;
    const N_SQ: usize = <() as Consts<N>>::N_SQ;

    pub fn from_stdin() -> Self {
        Self::from_reader(&mut std::io::BufReader::new(std::io::stdin()))
    }
    pub fn from_reader<R: std::io::BufRead>(reader: &mut R) -> Self {
        let data = std::array::from_fn(|_| {
            let mut reader_line = String::new();
            match reader.read_line(&mut reader_line) {
                Ok(n) => {
                    if n == 0 {
                        panic!("Unexpected EOF");
                    }
                },
                Err(e) => {
                    panic!("Error reading line: {}", e);
                }
            }
            let line = reader_line.trim_end();
            if line.len() != N {
                panic!("Line too short: expected {} characters, got {}", N, line.len());
            }
            line.chars().map(|c| {
                if c == consts::UNSOLVED {
                    bitset::Bitset::<N>::make_full()
                } else if let Some(num) = Self::ALPHABET.iter().position(|&x| x == c) {
                    bitset::Bitset::<N>::make_solved(num as Num)
                } else {
                    panic!("Invalid character in input: '{}'", c);
                }
            }).collect::<Vec<bitset::Bitset<N>>>().try_into().map_err(|_| "Invalid line length").unwrap()
        });
        Self { data: data }
    }

    pub fn to_stdout(&self) {
        for i in 0..N {
            for j in 0..N {
                let cell = &self.at(i, j);
                print!("{}", cell.to_printable_char());
                if j < N-1 {
                    print!(" ");
                    if (j % Self::N_SQ) == (Self::N_SQ - 1) {
                        print!(" ");
                    }
                }
            }
            println!();
            if (i % Self::N_SQ) == (Self::N_SQ - 1) {
                println!();
            }
        }
    }

    pub fn detailed_to_stdout(&self) {
        let n_times_nsq = N * Self::N_SQ;
        let mut data_sets: Vec<Vec<char>> = Vec::new(); // TODO: use arrays?
        for _ in 0..n_times_nsq {
            let row = vec!['x'; n_times_nsq];
            data_sets.push(row);
        }

        for r in 0..N {
            for c in 0..N {
                let cell = self.at(r, c);
                for n in 0..N {
                    data_sets[r * Self::N_SQ + n / Self::N_SQ][c * Self::N_SQ + n % Self::N_SQ]
                        = if cell.test(n as Num) { Self::ALPHABET[n] } else { ' ' };
                }
            }
        }

        for r in 0..data_sets.len() {
            let row = &data_sets[r];
            print!(" ");
            for c in 0..row.len() {
                print!("{}", row[c]);
                if c % N == (N - 1) {
                    if c < N * Self::N_SQ - 1 {
                        print!("  |||  ");
                    }
                } else if c % Self::N_SQ == Self::N_SQ - 1 {
                    print!(" | ");
                }
            }
            println!();
            let div_len: usize = match N {
                4 => 23,
                9 => 61,
                25 => 123,
                _ => panic!("div_size unknown")
            };
            if r % N == N - 1 {
                if r < N * Self::N_SQ - 1 {
                    println!();
                    for _ in 0..div_len { print!("="); }
                    println!();
                    println!();
                }
            } else if r % Self::N_SQ == Self::N_SQ - 1 {
                for _ in 0..div_len { print!("-"); }
                println!()
            }
	}
    }

    pub fn solve(&mut self) {
    }

    pub fn verify(&self) {
    }

    pub fn at(&self, i: usize, j: usize) -> &bitset::Bitset<N> {
        &self.data[i][j]
    }

    pub fn at_mut(&mut self, i: usize, j: usize) -> &mut bitset::Bitset<N> {
        &mut self.data[i][j]
    }
}


#[cfg(test)]
mod tests {
use super::Sudoku;


#[test]
fn test_load()
{
    {
        let input = concat!(
            "12345678*\n",
            "2345678**\n",
            "345678***\n",
            "45678****\n",
            "5678*****\n",
            "678******\n",
            "78*******\n",
            "8********\n",
            "*********\n"
        );
        let mut s = Sudoku::<9>::from_reader(&mut std::io::Cursor::new(input));
    }

    {
        let input = concat!(
            "1234\n",
            "12**\n",
            "2*4*\n",
            "**2*\n"
        );
        let mut s = Sudoku::<4>::from_reader(&mut std::io::Cursor::new(input));
    }
}

} // mod tests

