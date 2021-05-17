
std::vector<int> getTransitions(const GridLocation& loc, const int& direction) {

	std::vector<int> result(4);

	p::tuple possible_transitions = p::extract<p::tuple>(m_rail.attr("get_transitions")(loc.y, loc.x, direction));

	for (int dir = 0; dir < 4; dir++) {
		const char* currentValue = p::extract<char const *>(p::str(possible_transitions[dir]));

    result[dir] = boost::lexical_cast<int>(currentValue);
	}

	return result;
}