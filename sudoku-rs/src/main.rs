mod bitset;
mod consts;
mod sudoku;


fn main()
{
    let mut s = sudoku::Sudoku::<9>::from_stdin();
    println!("INPUT");
    s.to_stdout();
    println!();

    s.solve();

    println!("OUTPUT_DETAILED");
    s.detailed_to_stdout();
    println!();

    println!("OUTPUT");
    s.to_stdout();
    println!();

    s.verify();
}

