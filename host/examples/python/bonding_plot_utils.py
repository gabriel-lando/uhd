import matplotlib
import matplotlib.pyplot as plt
import os

def setup_academic_style():
    """Sets up the matplotlib rcParams for academic plotting style."""
    try:
        # Try to use seaborn colorblind style if available
        plt.style.use('seaborn-v0_8-colorblind')
    except OSError:
        pass # Fallback to default if not found
        
    matplotlib.rcParams.update({
        'font.family': 'serif',
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'legend.fontsize': 10,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'figure.figsize': (7, 4.5),
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'axes.grid': True,
        'grid.alpha': 0.3,
        'lines.linewidth': 1.2,
    })

def save_plot(fig, filename_base, output_dir="."):
    """Saves a figure to both PNG and PDF formats."""
    os.makedirs(output_dir, exist_ok=True)
    png_path = os.path.join(output_dir, f"{filename_base}.png")
    pdf_path = os.path.join(output_dir, f"{filename_base}.pdf")
    
    fig.savefig(png_path)
    fig.savefig(pdf_path)
    print(f"Saved {png_path} and {pdf_path}")
