import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import scipy.stats as stats
import os

def generate_combined_violin_plot(csv_file, rows=None, columns=None):
    # Read the CSV file
    data = pd.read_csv(csv_file)
    
    # Validate row indices
    if rows is not None:
        max_index = data.shape[0] - 1
        valid_rows = [row for row in rows if 0 <= row <= max_index]
        if not valid_rows:
            raise IndexError("No valid row indices provided.")
        data = data.iloc[valid_rows]
    
    # Validate column names
    if columns is not None:
        valid_columns = [col for col in columns if col in data.columns]
        if not valid_columns:
            raise ValueError("No valid column names provided.")
        data = data[valid_columns]
    
    # Create a directory to save the plot if it doesn't exist
    output_dir = 'plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Generate a combined violin plot for the specified columns
    plt.figure(figsize=(12, 8))
    sns.violinplot(data=data)
    plt.title('Aging Adult Questionare Violin Plots')
    plt.xticks(rotation=45, ha='right')  # Rotate x-axis labels
    plt.tight_layout()  # Adjust layout to make room for labels
    plt.savefig(f'{output_dir}/older_combined_violin_plot.png')
    plt.close()


def generate_box_and_whisker_plot(csv_file, rows=None, columns=None):
    # Read the CSV file
    data = pd.read_csv(csv_file)
    
    # Validate row indices
    if rows is not None:
        max_index = data.shape[0] - 1
        valid_rows = [row for row in rows if 0 <= row <= max_index]
        if not valid_rows:
            raise IndexError("No valid row indices provided.")
        data = data.iloc[valid_rows]
    
    # Validate column names
    if columns is not None:
        valid_columns = [col for col in columns if col in data.columns]
        if not valid_columns:
            raise ValueError("No valid column names provided.")
        data = data[valid_columns]
    
    # Create a directory to save the plot if it doesn't exist
    output_dir = 'plots'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Generate a combined box-and-whisker plot for the specified columns
    plt.figure(figsize=(12, 8))
    sns.boxplot(data=data)
    plt.title('Aging Adult Questionnaire Box-and-Whisker Plots')
    plt.xticks(rotation=45, ha='right')  # Rotate x-axis labels
    plt.tight_layout()  # Adjust layout to make room for labels
    plt.savefig(f'{output_dir}/older_combined_box_and_whisker_plot.png')
    plt.close()

def perform_t_test(csv_file, rows=None, column=None, popmean=4):
    # Read the CSV file
    data = pd.read_csv(csv_file)
    
    # Validate row indices
    if rows is not None:
        max_index = data.shape[0] - 1
        valid_rows = [row for row in rows if 0 <= row <= max_index]
        if not valid_rows:
            raise IndexError("No valid row indices provided.")
        data = data.iloc[valid_rows]
    
    # Validate column name
    if columns is not None:
        valid_columns = [col for col in columns if col in data.columns]
        if not valid_columns:
            raise ValueError("No valid column names provided.")
        data = data[valid_columns]
    
    # Perform the one-sample t-test
    t_stat, p_value = stats.ttest_1samp(data[column].dropna(), popmean)
    
    # Print the results
    print(f"One-Sample T-Test Results for {column}:")
    print(f"T-statistic: {t_stat}")
    print(f"P-value: {p_value}")

def independent_samples_t_test(csv_file, rows1=None, rows2=None, column=None):
   # Read the CSV file
    data = pd.read_csv(csv_file)

    
    # Validate row indices for first set
    if rows1 is not None:
        max_index = data.shape[0] - 1
        valid_rows1 = [row for row in rows1 if 0 <= row <= max_index]
        if not valid_rows1:
            raise IndexError("No valid row indices provided for the first set.")
        data1 = data.iloc[valid_rows1]
    
    # Validate row indices for second set
    if rows2 is not None:
        max_index = data.shape[0] - 1
        valid_rows2 = [row for row in rows2 if 0 <= row <= max_index]
        if not valid_rows2:
            raise IndexError("No valid row indices provided for the second set.")
        data2 = data.iloc[valid_rows2]
    
    # Validate column name
    if columns is not None:
        valid_columns = [col for col in columns if col in data.columns]
        if not valid_columns:
            raise ValueError("No valid column names provided.")
        data = data[valid_columns]
    
    # Perform the independent samples t-test
    t_stat, p_value = stats.ttest_ind(data1[column].dropna(), data2[column].dropna(), nan_policy='omit')
    
    # Print the results
    print(f"Independent Samples T-Test Results for {column}:")
    print(f"T-statistic: {t_stat}")
    print(f"P-value: {p_value}")


# Example usage
csv_file = 'ParticipantResponses.csv'  # Replace with your CSV file path
rows_pilot=[0,1,2,3,4,5,6,7,8,9]
rows_older = [10,11,12,13,14,15,16,17,18,19]  # Specify the row indices you want to include
columns = ['Understandability (Human to Robot)','Understandability (Robot to Human)','Responsive' ,'Friendly','Awkwardness','Competency','Intelligence','Coherent (Exercise Related)','Compassionate (Exercise Related)']  # Specify the column names you want to include
generate_combined_violin_plot(csv_file, rows_older ,columns)
generate_box_and_whisker_plot(csv_file,rows_older,columns)
#perform_t_test(csv_file,rows_older,columns)
#perform_t_test(csv_file,rows_pilot,columns)
#independent_samples_t_test(csv_file,rows_pilot,rows_older,columns)
