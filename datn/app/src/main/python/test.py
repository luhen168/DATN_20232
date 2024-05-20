def process_data(data):
    splitted_data = data.split(";")
    processed_data = "\n\n".join(splitted_data)
    return "Processed data:\n" + processed_data
